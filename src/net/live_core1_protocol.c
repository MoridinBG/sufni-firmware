#include "live_core1_protocol.h"

#include "live_protocol.h"
#include "tcpserver.h"

#include "../fw/live_stream_shared.h"
#include "../fw/live_watchdog_diag.h"
#include "../fw/sensor_setup.h"
#include "../util/log.h"

#include "lwip/tcp.h"
#include "pico/async_context.h"
#include "pico/async_context_threadsafe_background.h"
#include "pico/cyw43_arch.h"
#include "pico/error.h"
#include "pico/time.h"
#include "pico/unique_id.h"

#include <string.h>

#define LIVE_LWIP_LOCK_WAIT_US         2000u
#define LIVE_SESSION_STATS_INTERVAL_US 500000u
#define LIVE_TX_ABORT_TIMEOUT_US       1000000u
#define LIVE_TX_MAX_BURST_FRAMES       4u

#define LIVE_TX_FRAME_BUFFER_SIZE                                                                                      \
    (sizeof(struct live_frame_header) + sizeof(struct live_batch_payload) +                                            \
     (LIVE_IMU_RECORDS_PER_SLOT * sizeof(struct imu_record)))

static uint8_t live_tx_frame_buffer[LIVE_TX_FRAME_BUFFER_SIZE];

static bool live_protocol_can_accept(const struct tcpserver *server);
static void live_protocol_on_accept(struct tcpserver *server);
static void live_protocol_on_disconnect(struct tcpserver *server);
static bool live_protocol_detect(const struct tcpserver *server);
static bool live_protocol_process_rx(struct tcpserver *server);
static bool live_protocol_service(struct tcpserver *server);
static bool live_protocol_needs_service(const struct tcpserver *server);
static void live_protocol_wait_for_quiescent_control(struct tcpserver *server);
static void live_protocol_abort(struct tcpserver *server);
static bool live_send_session_stats(struct tcpserver *server);

enum live_tx_diag_failure_reason {
    LIVE_TX_DIAG_FAIL_LWIP_LOCK = 0,
    LIVE_TX_DIAG_FAIL_SNDBUF,
    LIVE_TX_DIAG_FAIL_WRITE_ERR,
};

enum live_tx_diag_stream_index {
    LIVE_TX_DIAG_STREAM_TRAVEL = 0,
    LIVE_TX_DIAG_STREAM_IMU,
    LIVE_TX_DIAG_STREAM_GPS,
    LIVE_TX_DIAG_STREAM_COUNT,
};

enum live_write_result {
    LIVE_WRITE_RESULT_OK = 0,
    LIVE_WRITE_RESULT_RETRY,
    LIVE_WRITE_RESULT_DROP,
};

enum live_send_slot_result {
    LIVE_SEND_SLOT_NONE = 0,
    LIVE_SEND_SLOT_SENT,
    LIVE_SEND_SLOT_DROPPED,
    LIVE_SEND_SLOT_BLOCKED,
};

static uint64_t live_tx_diag_age_us(uint64_t now_us, uint64_t timestamp_us) {
    if (timestamp_us == 0u) {
        return 0u;
    }

    return now_us - timestamp_us;
}

static void live_tx_diag_note_pcb_state(struct tcpserver *server, const struct tcp_pcb *pcb, uint32_t sndbuf) {
    if (server == NULL || pcb == NULL) {
        return;
    }

    server->live.tx_last_sndbuf = sndbuf;
    if (sndbuf < server->live.tx_min_sndbuf) {
        server->live.tx_min_sndbuf = sndbuf;
    }

    server->live.tx_last_sndqueuelen = pcb->snd_queuelen;
    if (pcb->snd_queuelen > server->live.tx_max_sndqueuelen) {
        server->live.tx_max_sndqueuelen = pcb->snd_queuelen;
    }
}

static void live_tx_diag_note_write_err_code(struct tcpserver *server, err_t err) {
    if (server == NULL) {
        return;
    }

    server->live.tx_last_write_err = (int8_t)err;

    switch (err) {
        case ERR_MEM:
            server->live.tx_fail_err_mem++;
            break;
        case ERR_CONN:
            server->live.tx_fail_err_conn++;
            break;
        case ERR_ABRT:
            server->live.tx_fail_err_abrt++;
            break;
        case ERR_RST:
            server->live.tx_fail_err_rst++;
            break;
        case ERR_CLSD:
            server->live.tx_fail_err_clsd++;
            break;
        case ERR_VAL:
            server->live.tx_fail_err_val++;
            break;
        default:
            server->live.tx_fail_err_other++;
            break;
    }
}

static void live_tx_diag_note_success(struct tcpserver *server, enum live_tx_diag_frame_kind frame_kind,
                                      uint32_t total_bytes) {
    uint64_t now_us;

    if (frame_kind >= LIVE_TX_DIAG_FRAME_COUNT) {
        return;
    }

    now_us = time_us_64();
    server->live.tx_ok_count[frame_kind]++;
    server->live.tx_last_ok_us[frame_kind] = now_us;
    server->live.tx_enqueued_bytes += total_bytes;
}

static void live_tx_diag_note_failure(struct tcpserver *server, enum live_tx_diag_frame_kind frame_kind,
                                      enum live_tx_diag_failure_reason reason) {
    uint64_t now_us;

    if (frame_kind >= LIVE_TX_DIAG_FRAME_COUNT) {
        return;
    }

    now_us = time_us_64();
    server->live.tx_fail_count[frame_kind]++;
    server->live.tx_last_fail_us[frame_kind] = now_us;

    switch (reason) {
        case LIVE_TX_DIAG_FAIL_LWIP_LOCK:
            server->live.tx_fail_lwip_lock++;
            break;
        case LIVE_TX_DIAG_FAIL_SNDBUF:
            server->live.tx_fail_sndbuf++;
            break;
        case LIVE_TX_DIAG_FAIL_WRITE_ERR:
            server->live.tx_fail_write_err++;
            break;
    }
}

static volatile uint32_t *live_dropped_batches_counter(enum live_tx_diag_frame_kind frame_kind) {
    switch (frame_kind) {
        case LIVE_TX_DIAG_FRAME_TRAVEL:
            return &live_stream_shared.travel_stats.dropped_batches;
        case LIVE_TX_DIAG_FRAME_IMU:
            return &live_stream_shared.imu_stats.dropped_batches;
        case LIVE_TX_DIAG_FRAME_GPS:
            return &live_stream_shared.gps_stats.dropped_batches;
        default:
            return NULL;
    }
}

static const char *live_frame_kind_name(enum live_tx_diag_frame_kind frame_kind) {
    switch (frame_kind) {
        case LIVE_TX_DIAG_FRAME_TRAVEL:
            return "travel";
        case LIVE_TX_DIAG_FRAME_IMU:
            return "imu";
        case LIVE_TX_DIAG_FRAME_GPS:
            return "gps";
        case LIVE_TX_DIAG_FRAME_SESSION_STATS:
            return "stats";
        case LIVE_TX_DIAG_FRAME_CONTROL:
            return "control";
        default:
            return "unknown";
    }
}

static void live_log_backpressure_drop(struct tcpserver *server, enum live_tx_diag_frame_kind frame_kind,
                                       const struct live_slot_header *header) {
    uint64_t now_us = time_us_64();

    server->live.tx_backpressure_drop_count++;
    if (server->live.tx_last_drop_log_us != 0u && (now_us - server->live.tx_last_drop_log_us) < 500000u) {
        return;
    }

    LOG("LIVE", "Dropping stale %s batch seq=%u samples=%u age_us=%llu sndbuf=%u sndq=%u drops=%u\n",
        live_frame_kind_name(frame_kind), (unsigned)header->sequence, (unsigned)header->sample_count,
        (unsigned long long)live_tx_diag_age_us(now_us, header->first_monotonic_us),
        (unsigned)server->live.tx_last_sndbuf, (unsigned)server->live.tx_last_sndqueuelen,
        (unsigned)server->live.tx_backpressure_drop_count);
    server->live.tx_last_drop_log_us = now_us;
}

static bool live_tx_diag_has_failures(const struct tcpserver *server) {
    uint32_t frame_kind;

    if (server->live.tx_fail_lwip_lock > 0u || server->live.tx_fail_sndbuf > 0u ||
        server->live.tx_fail_write_err > 0u || server->live.tx_timeout_fired) {
        return true;
    }

    for (frame_kind = 0; frame_kind < LIVE_TX_DIAG_FRAME_COUNT; ++frame_kind) {
        if (server->live.tx_fail_count[frame_kind] > 0u) {
            return true;
        }
    }

    return false;
}

static uint32_t live_tx_diag_ready_depth_and_oldest_age(void *base, size_t stride, uint32_t slot_count, uint64_t now_us,
                                                        uint64_t *oldest_age_us) {
    uint32_t slot_index;
    uint32_t depth = 0u;
    uint64_t oldest_age = 0u;

    for (slot_index = 0; slot_index < slot_count; ++slot_index) {
        struct live_slot_header *header = (struct live_slot_header *)((uint8_t *)base + (stride * slot_index));
        uint64_t age_us;

        if (header->state != LIVE_SLOT_READY) {
            continue;
        }

        depth++;
        age_us = live_tx_diag_age_us(now_us, header->first_monotonic_us);
        if (age_us > oldest_age) {
            oldest_age = age_us;
        }
    }

    if (oldest_age_us != NULL) {
        *oldest_age_us = oldest_age;
    }

    return depth;
}

static void live_tx_diag_update_queue_state(struct tcpserver *server) {
    uint64_t now_us;
    uint64_t oldest_age_us;
    uint32_t depth;

    if (server == NULL) {
        return;
    }

    now_us = time_us_64();

    depth = live_tx_diag_ready_depth_and_oldest_age(live_stream_shared.travel_slots, sizeof(struct live_travel_slot),
                                                    LIVE_TRAVEL_SLOT_COUNT, now_us, &oldest_age_us);
    server->live.tx_ready_depth_last[LIVE_TX_DIAG_STREAM_TRAVEL] = depth;
    if (depth > server->live.tx_ready_depth_max[LIVE_TX_DIAG_STREAM_TRAVEL]) {
        server->live.tx_ready_depth_max[LIVE_TX_DIAG_STREAM_TRAVEL] = depth;
    }
    server->live.tx_ready_oldest_age_last_us[LIVE_TX_DIAG_STREAM_TRAVEL] = oldest_age_us;
    if (oldest_age_us > server->live.tx_ready_oldest_age_max_us[LIVE_TX_DIAG_STREAM_TRAVEL]) {
        server->live.tx_ready_oldest_age_max_us[LIVE_TX_DIAG_STREAM_TRAVEL] = oldest_age_us;
    }

    depth = live_tx_diag_ready_depth_and_oldest_age(live_stream_shared.imu_slots, sizeof(struct live_imu_slot),
                                                    LIVE_IMU_SLOT_COUNT, now_us, &oldest_age_us);
    server->live.tx_ready_depth_last[LIVE_TX_DIAG_STREAM_IMU] = depth;
    if (depth > server->live.tx_ready_depth_max[LIVE_TX_DIAG_STREAM_IMU]) {
        server->live.tx_ready_depth_max[LIVE_TX_DIAG_STREAM_IMU] = depth;
    }
    server->live.tx_ready_oldest_age_last_us[LIVE_TX_DIAG_STREAM_IMU] = oldest_age_us;
    if (oldest_age_us > server->live.tx_ready_oldest_age_max_us[LIVE_TX_DIAG_STREAM_IMU]) {
        server->live.tx_ready_oldest_age_max_us[LIVE_TX_DIAG_STREAM_IMU] = oldest_age_us;
    }

    depth = live_tx_diag_ready_depth_and_oldest_age(live_stream_shared.gps_slots, sizeof(struct live_gps_slot),
                                                    LIVE_GPS_SLOT_COUNT, now_us, &oldest_age_us);
    server->live.tx_ready_depth_last[LIVE_TX_DIAG_STREAM_GPS] = depth;
    if (depth > server->live.tx_ready_depth_max[LIVE_TX_DIAG_STREAM_GPS]) {
        server->live.tx_ready_depth_max[LIVE_TX_DIAG_STREAM_GPS] = depth;
    }
    server->live.tx_ready_oldest_age_last_us[LIVE_TX_DIAG_STREAM_GPS] = oldest_age_us;
    if (oldest_age_us > server->live.tx_ready_oldest_age_max_us[LIVE_TX_DIAG_STREAM_GPS]) {
        server->live.tx_ready_oldest_age_max_us[LIVE_TX_DIAG_STREAM_GPS] = oldest_age_us;
    }
}

static void live_log_tx_diag_summary(struct tcpserver *server, const char *reason) {
    uint64_t now_us;

    if (server == NULL || server->live.tx_diag_summary_logged || !live_tx_diag_has_failures(server)) {
        return;
    }

    live_tx_diag_update_queue_state(server);
    now_us = time_us_64();

    LOG("LIVE", "TXD1 rsn=%s flg=%u,%u,%u lw=%u sb=%u we=%u ok=%u,%u,%u,%u,%u fail=%u,%u,%u,%u,%u\n", reason,
        (unsigned)server->live.tx_timeout_fired, (unsigned)server->live.tx_timeout_requested_stop,
        (unsigned)server->live.tx_timeout_requested_close, (unsigned)server->live.tx_fail_lwip_lock,
        (unsigned)server->live.tx_fail_sndbuf, (unsigned)server->live.tx_fail_write_err,
        (unsigned)server->live.tx_ok_count[LIVE_TX_DIAG_FRAME_TRAVEL],
        (unsigned)server->live.tx_ok_count[LIVE_TX_DIAG_FRAME_IMU],
        (unsigned)server->live.tx_ok_count[LIVE_TX_DIAG_FRAME_GPS],
        (unsigned)server->live.tx_ok_count[LIVE_TX_DIAG_FRAME_SESSION_STATS],
        (unsigned)server->live.tx_ok_count[LIVE_TX_DIAG_FRAME_CONTROL],
        (unsigned)server->live.tx_fail_count[LIVE_TX_DIAG_FRAME_TRAVEL],
        (unsigned)server->live.tx_fail_count[LIVE_TX_DIAG_FRAME_IMU],
        (unsigned)server->live.tx_fail_count[LIVE_TX_DIAG_FRAME_GPS],
        (unsigned)server->live.tx_fail_count[LIVE_TX_DIAG_FRAME_SESSION_STATS],
        (unsigned)server->live.tx_fail_count[LIVE_TX_DIAG_FRAME_CONTROL]);
    LOG("LIVE", "TXD2 ok=%llu,%llu,%llu,%llu,%llu fail=%llu,%llu,%llu,%llu,%llu\n",
        (unsigned long long)live_tx_diag_age_us(now_us, server->live.tx_last_ok_us[LIVE_TX_DIAG_FRAME_TRAVEL]),
        (unsigned long long)live_tx_diag_age_us(now_us, server->live.tx_last_ok_us[LIVE_TX_DIAG_FRAME_IMU]),
        (unsigned long long)live_tx_diag_age_us(now_us, server->live.tx_last_ok_us[LIVE_TX_DIAG_FRAME_GPS]),
        (unsigned long long)live_tx_diag_age_us(now_us, server->live.tx_last_ok_us[LIVE_TX_DIAG_FRAME_SESSION_STATS]),
        (unsigned long long)live_tx_diag_age_us(now_us, server->live.tx_last_ok_us[LIVE_TX_DIAG_FRAME_CONTROL]),
        (unsigned long long)live_tx_diag_age_us(now_us, server->live.tx_last_fail_us[LIVE_TX_DIAG_FRAME_TRAVEL]),
        (unsigned long long)live_tx_diag_age_us(now_us, server->live.tx_last_fail_us[LIVE_TX_DIAG_FRAME_IMU]),
        (unsigned long long)live_tx_diag_age_us(now_us, server->live.tx_last_fail_us[LIVE_TX_DIAG_FRAME_GPS]),
        (unsigned long long)live_tx_diag_age_us(now_us, server->live.tx_last_fail_us[LIVE_TX_DIAG_FRAME_SESSION_STATS]),
        (unsigned long long)live_tx_diag_age_us(now_us, server->live.tx_last_fail_us[LIVE_TX_DIAG_FRAME_CONTROL]));
    LOG("LIVE", "TXD3 err=%d em=%u cn=%u ab=%u rs=%u cl=%u va=%u ot=%u sq=%u/%u sb=%u/%u ack=%u,%llu,%u by=%llu/%llu\n",
        (int)server->live.tx_last_write_err, (unsigned)server->live.tx_fail_err_mem,
        (unsigned)server->live.tx_fail_err_conn, (unsigned)server->live.tx_fail_err_abrt,
        (unsigned)server->live.tx_fail_err_rst, (unsigned)server->live.tx_fail_err_clsd,
        (unsigned)server->live.tx_fail_err_val, (unsigned)server->live.tx_fail_err_other,
        (unsigned)server->live.tx_last_sndqueuelen, (unsigned)server->live.tx_max_sndqueuelen,
        (unsigned)server->live.tx_last_sndbuf,
        (unsigned)(server->live.tx_min_sndbuf == UINT32_MAX ? 0u : server->live.tx_min_sndbuf),
        (unsigned)server->live.tx_ack_events,
        (unsigned long long)live_tx_diag_age_us(now_us, server->live.tx_last_ack_us),
        (unsigned)server->live.tx_last_ack_len, (unsigned long long)server->live.tx_acked_bytes,
        (unsigned long long)server->live.tx_enqueued_bytes);
    LOG("LIVE",
        "TXD4 prod=%llu,%llu,%llu sent=%u,%u,%u drop=%u,%u,%u q=%u/%u,%u/%u,%u/%u age=%llu/%llu,%llu/%llu,%llu/%llu\n",
        (unsigned long long)live_stream_shared.travel_stats.next_sequence,
        (unsigned long long)live_stream_shared.imu_stats.next_sequence,
        (unsigned long long)live_stream_shared.gps_stats.next_sequence,
        (unsigned)server->live.tx_ok_count[LIVE_TX_DIAG_FRAME_TRAVEL],
        (unsigned)server->live.tx_ok_count[LIVE_TX_DIAG_FRAME_IMU],
        (unsigned)server->live.tx_ok_count[LIVE_TX_DIAG_FRAME_GPS],
        (unsigned)live_stream_shared.travel_stats.dropped_batches,
        (unsigned)live_stream_shared.imu_stats.dropped_batches, (unsigned)live_stream_shared.gps_stats.dropped_batches,
        (unsigned)server->live.tx_ready_depth_last[LIVE_TX_DIAG_STREAM_TRAVEL],
        (unsigned)server->live.tx_ready_depth_max[LIVE_TX_DIAG_STREAM_TRAVEL],
        (unsigned)server->live.tx_ready_depth_last[LIVE_TX_DIAG_STREAM_IMU],
        (unsigned)server->live.tx_ready_depth_max[LIVE_TX_DIAG_STREAM_IMU],
        (unsigned)server->live.tx_ready_depth_last[LIVE_TX_DIAG_STREAM_GPS],
        (unsigned)server->live.tx_ready_depth_max[LIVE_TX_DIAG_STREAM_GPS],
        (unsigned long long)server->live.tx_ready_oldest_age_last_us[LIVE_TX_DIAG_STREAM_TRAVEL],
        (unsigned long long)server->live.tx_ready_oldest_age_max_us[LIVE_TX_DIAG_STREAM_TRAVEL],
        (unsigned long long)server->live.tx_ready_oldest_age_last_us[LIVE_TX_DIAG_STREAM_IMU],
        (unsigned long long)server->live.tx_ready_oldest_age_max_us[LIVE_TX_DIAG_STREAM_IMU],
        (unsigned long long)server->live.tx_ready_oldest_age_last_us[LIVE_TX_DIAG_STREAM_GPS],
        (unsigned long long)server->live.tx_ready_oldest_age_max_us[LIVE_TX_DIAG_STREAM_GPS]);

    server->live.tx_diag_summary_logged = true;
}

static void live_reset_tx_diag(struct tcpserver *server) {
    server->live.tx_failure_active = false;
    server->live.tx_failure_started_us = 0;
}

static void live_note_tx_failure(struct tcpserver *server) {
    if (!server->live.tx_failure_active) {
        server->live.tx_failure_active = true;
        server->live.tx_failure_started_us = time_us_64();
    }
}

static void live_clear_tx_failure(struct tcpserver *server) {
    if (!server->live.tx_failure_active) {
        return;
    }

    server->live.tx_failure_active = false;
    server->live.tx_failure_started_us = 0;
}

static bool live_tx_stall_timed_out(const struct tcpserver *server) {
    uint64_t deadline_start_us;

    if (!server->live.tx_failure_active) {
        return false;
    }

    deadline_start_us = server->live.tx_failure_started_us;
    if (server->live.tx_last_ack_us >= deadline_start_us) {
        deadline_start_us = server->live.tx_last_ack_us;
    }

    return (time_us_64() - deadline_start_us) >= LIVE_TX_ABORT_TIMEOUT_US;
}

static bool live_has_ready_backlog(const struct tcpserver *server) {
    return server->live.tx_ready_depth_last[LIVE_TX_DIAG_STREAM_TRAVEL] > 1u ||
           server->live.tx_ready_depth_last[LIVE_TX_DIAG_STREAM_IMU] > 1u ||
           server->live.tx_ready_depth_last[LIVE_TX_DIAG_STREAM_GPS] > 1u;
}

static void live_maybe_send_session_stats(struct tcpserver *server, uint64_t now_us) {
    if ((now_us - server->live.last_stats_us) < LIVE_SESSION_STATS_INTERVAL_US) {
        return;
    }

    // Under backpressure, failed stats sends can devolve into a per-tick retry storm that
    // consumes lwIP lock time and worsens ERR_MEM churn without helping the data path recover.
    if (server->live.tx_failure_active || live_has_ready_backlog(server)) {
        server->live.last_stats_us = now_us;
        return;
    }

    (void)live_send_session_stats(server);
    server->live.last_stats_us = now_us;
}

static bool live_try_enter_lwip(async_context_t **context_out) {
    async_context_t *context = cyw43_arch_async_context();

    if (context == NULL) {
        return false;
    }

#if PICO_CYW43_ARCH_THREADSAFE_BACKGROUND
    {
        async_context_threadsafe_background_t *threadsafe = (async_context_threadsafe_background_t *)context;
        absolute_time_t deadline = make_timeout_time_us(LIVE_LWIP_LOCK_WAIT_US);

        do {
            if (recursive_mutex_try_enter(&threadsafe->lock_mutex, NULL)) {
                *context_out = context;
                return true;
            }
            live_watchdog_diag_mark_core1(LIVE_WATCHDOG_CORE1_MARKER_LWIP_LOCK_WAIT);
            tight_loop_contents();
        } while (!time_reached(deadline));

        return false;
    }
#else
    cyw43_arch_lwip_begin();
    *context_out = context;
    return true;
#endif
}

const struct tcpserver_protocol_ops live_core1_protocol_ops = {
    .can_accept = live_protocol_can_accept,
    .on_accept = live_protocol_on_accept,
    .on_disconnect = live_protocol_on_disconnect,
    .detect = live_protocol_detect,
    .process_rx = live_protocol_process_rx,
    .service = live_protocol_service,
    .needs_service = live_protocol_needs_service,
};

static void live_reset_session(struct tcpserver *server) {
    uint64_t now_us = time_us_64();

    server->live = (struct live_protocol_session){
        .tx_sequence = 1,
        .last_stats_us = now_us,
        .tx_last_write_err = 0,
        .tx_min_sndbuf = UINT32_MAX,
    };
    live_reset_tx_diag(server);
}

static enum live_write_result live_write_frame_bytes(struct tcpserver *server, enum live_tx_diag_frame_kind frame_kind,
                                                     const void *frame_bytes, uint32_t total_bytes) {
    async_context_t *lwip_context = NULL;
    err_t err;
    uint32_t sndbuf;

    if (server->client_pcb == NULL) {
        return LIVE_WRITE_RESULT_RETRY;
    }

    live_watchdog_diag_mark_core1(LIVE_WATCHDOG_CORE1_MARKER_WRITE_FRAME);

    if (!live_try_enter_lwip(&lwip_context)) {
        live_tx_diag_note_failure(server, frame_kind, LIVE_TX_DIAG_FAIL_LWIP_LOCK);
        live_note_tx_failure(server);
        return LIVE_WRITE_RESULT_RETRY;
    }
    sndbuf = tcp_sndbuf(server->client_pcb);
    live_tx_diag_note_pcb_state(server, server->client_pcb, sndbuf);
    if (sndbuf < total_bytes) {
        async_context_release_lock(lwip_context);
        live_tx_diag_note_failure(server, frame_kind, LIVE_TX_DIAG_FAIL_SNDBUF);
        live_clear_tx_failure(server);
        return LIVE_WRITE_RESULT_DROP;
    }

    err = tcp_write(server->client_pcb, frame_bytes, total_bytes, TCP_WRITE_FLAG_COPY);
    if (err == ERR_OK) {
        tcp_output(server->client_pcb);
        live_tx_diag_note_pcb_state(server, server->client_pcb, tcp_sndbuf(server->client_pcb));
    }
    async_context_release_lock(lwip_context);

    if (err != ERR_OK) {
        live_tx_diag_note_failure(server, frame_kind, LIVE_TX_DIAG_FAIL_WRITE_ERR);
        live_tx_diag_note_write_err_code(server, err);
        if (err == ERR_MEM) {
            live_clear_tx_failure(server);
            return LIVE_WRITE_RESULT_DROP;
        }

        live_note_tx_failure(server);
        return LIVE_WRITE_RESULT_RETRY;
    }

    live_tx_diag_note_success(server, frame_kind, total_bytes);
    live_clear_tx_failure(server);

    return LIVE_WRITE_RESULT_OK;
}

static bool live_send_frame(struct tcpserver *server, enum live_tx_diag_frame_kind frame_kind, uint16_t frame_type,
                            const void *payload, uint32_t payload_length, uint32_t sequence) {
    struct live_frame_header header = {
        .magic = LIVE_PROTOCOL_MAGIC,
        .version = LIVE_PROTOCOL_VERSION,
        .frame_type = frame_type,
        .payload_length = payload_length,
        .sequence = sequence,
    };
    uint32_t total_bytes = sizeof(header) + payload_length;
    if (total_bytes > sizeof(live_tx_frame_buffer)) {
        return false;
    }

    memcpy(live_tx_frame_buffer, &header, sizeof(header));
    if (payload_length > 0u) {
        memcpy(live_tx_frame_buffer + sizeof(header), payload, payload_length);
    }

    if (live_write_frame_bytes(server, frame_kind, live_tx_frame_buffer, total_bytes) != LIVE_WRITE_RESULT_OK) {
        return false;
    }

    return true;
}

static bool live_send_error(struct tcpserver *server, int32_t error_code) {
    uint32_t sequence = server->live.tx_sequence;
    struct live_error_frame frame = {.error_code = error_code};

    if (!live_send_frame(server, LIVE_TX_DIAG_FRAME_CONTROL, LIVE_FRAME_ERROR, &frame, sizeof(frame), sequence)) {
        return false;
    }

    server->live.tx_sequence++;
    return true;
}

static bool live_send_start_ack(struct tcpserver *server) {
    uint32_t sequence = server->live.tx_sequence;
    struct live_start_ack_frame frame = {
        .result = live_stream_shared.start_response.result,
        .session_id = live_stream_shared.start_response.session_id,
        .selected_sensor_mask = live_stream_shared.start_response.selected_sensor_mask,
    };

    if (!live_send_frame(server, LIVE_TX_DIAG_FRAME_CONTROL, LIVE_FRAME_START_LIVE_ACK, &frame, sizeof(frame),
                         sequence)) {
        return false;
    }

    server->live.tx_sequence++;
    return true;
}

static bool live_send_stop_ack(struct tcpserver *server) {
    uint32_t sequence = server->live.tx_sequence;
    struct live_stop_ack_frame frame = {
        .session_id = live_stream_shared.start_response.session_id,
    };

    if (!live_send_frame(server, LIVE_TX_DIAG_FRAME_CONTROL, LIVE_FRAME_STOP_LIVE_ACK, &frame, sizeof(frame),
                         sequence)) {
        return false;
    }

    server->live.tx_sequence++;
    return true;
}

static bool live_send_session_header(struct tcpserver *server) {
    uint32_t sequence = server->live.tx_sequence;
    struct live_session_header_frame frame = {
        .session_id = live_stream_shared.start_response.session_id,
        .accepted_travel_hz = live_stream_shared.start_response.accepted_travel_hz,
        .accepted_imu_hz = live_stream_shared.start_response.accepted_imu_hz,
        .accepted_gps_fix_hz = live_stream_shared.start_response.accepted_gps_fix_hz,
        .session_start_utc = live_stream_shared.start_response.session_start_utc,
        .session_start_monotonic_us = live_stream_shared.start_response.session_start_monotonic_us,
        .active_imu_mask = live_stream_shared.start_response.active_imu_mask,
        .flags = LIVE_SESSION_FLAG_CALIBRATED_ONLY | LIVE_SESSION_FLAG_MUTUALLY_EXCLUSIVE_WITH_RECORDING,
        .requested_sensor_mask = live_stream_shared.start_response.requested_sensor_mask,
        .accepted_sensor_mask = live_stream_shared.start_response.accepted_sensor_mask,
    };

#if HAS_IMU
    if (imu_frame.available) {
        frame.imu_accel_lsb_per_g[0] = imu_frame.accel_lsb_per_g;
        frame.imu_gyro_lsb_per_dps[0] = imu_frame.gyro_lsb_per_dps;
    }
    if (imu_fork.available) {
        frame.imu_accel_lsb_per_g[1] = imu_fork.accel_lsb_per_g;
        frame.imu_gyro_lsb_per_dps[1] = imu_fork.gyro_lsb_per_dps;
    }
    if (imu_rear.available) {
        frame.imu_accel_lsb_per_g[2] = imu_rear.accel_lsb_per_g;
        frame.imu_gyro_lsb_per_dps[2] = imu_rear.gyro_lsb_per_dps;
    }
#endif

    if (!live_send_frame(server, LIVE_TX_DIAG_FRAME_CONTROL, LIVE_FRAME_SESSION_HEADER, &frame, sizeof(frame),
                         sequence)) {
        return false;
    }

    server->live.tx_sequence++;
    return true;
}

static bool live_send_session_stats(struct tcpserver *server) {
    uint32_t sequence = server->live.tx_sequence;
    struct live_session_stats_frame frame = {
        .session_id = live_stream_shared.start_response.session_id,
        .travel_queue_depth = live_stream_shared.travel_stats.queue_depth,
        .imu_queue_depth = live_stream_shared.imu_stats.queue_depth,
        .gps_queue_depth = live_stream_shared.gps_stats.queue_depth,
        .travel_dropped_batches = live_stream_shared.travel_stats.dropped_batches,
        .imu_dropped_batches = live_stream_shared.imu_stats.dropped_batches,
        .gps_dropped_batches = live_stream_shared.gps_stats.dropped_batches,
    };

    if (!live_send_frame(server, LIVE_TX_DIAG_FRAME_SESSION_STATS, LIVE_FRAME_SESSION_STATS, &frame, sizeof(frame),
                         sequence)) {
        return false;
    }

    server->live.tx_sequence++;
    return true;
}

// Atomically claim the oldest READY slot for sending. Returns -1 if none available.
// Core 0 owns FREE→FILLING→READY; this side owns READY→SENDING→FREE.
static int live_claim_oldest_ready_slot(void *base, size_t stride, uint32_t slot_count) {
    uint32_t slot_index;
    uint32_t oldest_sequence = UINT32_MAX;
    int oldest_ready_index = -1;

    for (slot_index = 0; slot_index < slot_count; ++slot_index) {
        struct live_slot_header *header = (struct live_slot_header *)((uint8_t *)base + (stride * slot_index));
        if (header->state == LIVE_SLOT_READY && header->sequence <= oldest_sequence) {
            oldest_sequence = header->sequence;
            oldest_ready_index = (int)slot_index;
        }
    }

    if (oldest_ready_index >= 0) {
        struct live_slot_header *header =
            (struct live_slot_header *)((uint8_t *)base + (stride * (uint32_t)oldest_ready_index));
        uint32_t expected_state = LIVE_SLOT_READY;
        if (__atomic_compare_exchange_n(&header->state, &expected_state, LIVE_SLOT_SENDING, false, __ATOMIC_ACQ_REL,
                                        __ATOMIC_ACQUIRE)) {
            __dmb();
            return oldest_ready_index;
        }
    }

    return -1;
}

static enum live_send_slot_result live_send_slot_frame(struct tcpserver *server,
                                                       enum live_tx_diag_frame_kind frame_kind, uint16_t frame_type,
                                                       struct live_slot_header *header, const void *payload) {
    struct live_batch_payload batch = {
        .session_id = live_stream_shared.start_response.session_id,
        .stream_sequence = header->sequence,
        .first_index = header->first_index,
        .first_monotonic_us = header->first_monotonic_us,
        .sample_count = header->sample_count,
    };
    struct live_frame_header frame_header = {
        .magic = LIVE_PROTOCOL_MAGIC,
        .version = LIVE_PROTOCOL_VERSION,
        .frame_type = frame_type,
        .payload_length = sizeof(batch) + header->payload_bytes,
        .sequence = server->live.tx_sequence,
    };
    uint32_t total_bytes = sizeof(frame_header) + sizeof(batch) + header->payload_bytes;
    volatile uint32_t *dropped_batches;
    enum live_write_result write_result;

    if (total_bytes > sizeof(live_tx_frame_buffer)) {
        __atomic_store_n(&header->state, LIVE_SLOT_READY, __ATOMIC_RELEASE);
        return LIVE_SEND_SLOT_BLOCKED;
    }

    memcpy(live_tx_frame_buffer, &frame_header, sizeof(frame_header));
    memcpy(live_tx_frame_buffer + sizeof(frame_header), &batch, sizeof(batch));
    if (header->payload_bytes > 0u) {
        memcpy(live_tx_frame_buffer + sizeof(frame_header) + sizeof(batch), payload, header->payload_bytes);
    }

    write_result = live_write_frame_bytes(server, frame_kind, live_tx_frame_buffer, total_bytes);
    if (write_result != LIVE_WRITE_RESULT_OK) {
        if (write_result == LIVE_WRITE_RESULT_DROP) {
            dropped_batches = live_dropped_batches_counter(frame_kind);
            if (dropped_batches != NULL) {
                (*dropped_batches)++;
            }
            live_log_backpressure_drop(server, frame_kind, header);
            live_stream_release_slot(&header->state);
            return LIVE_SEND_SLOT_DROPPED;
        }

        __atomic_store_n(&header->state, LIVE_SLOT_READY, __ATOMIC_RELEASE);
        return LIVE_SEND_SLOT_BLOCKED;
    }

    server->live.tx_sequence++;
    live_stream_release_slot(&header->state);
    return LIVE_SEND_SLOT_SENT;
}

static enum live_send_slot_result live_send_next_ready_slot(struct tcpserver *server) {
    int slot_index;

    slot_index = live_claim_oldest_ready_slot(live_stream_shared.travel_slots, sizeof(struct live_travel_slot),
                                              LIVE_TRAVEL_SLOT_COUNT);
    if (slot_index >= 0) {
        struct live_travel_slot *slot = &live_stream_shared.travel_slots[slot_index];
        return live_send_slot_frame(server, LIVE_TX_DIAG_FRAME_TRAVEL, LIVE_FRAME_TRAVEL_BATCH, &slot->header,
                                    slot->payload);
    }

    slot_index =
        live_claim_oldest_ready_slot(live_stream_shared.imu_slots, sizeof(struct live_imu_slot), LIVE_IMU_SLOT_COUNT);
    if (slot_index >= 0) {
        struct live_imu_slot *slot = &live_stream_shared.imu_slots[slot_index];
        return live_send_slot_frame(server, LIVE_TX_DIAG_FRAME_IMU, LIVE_FRAME_IMU_BATCH, &slot->header, slot->payload);
    }

    slot_index =
        live_claim_oldest_ready_slot(live_stream_shared.gps_slots, sizeof(struct live_gps_slot), LIVE_GPS_SLOT_COUNT);
    if (slot_index >= 0) {
        struct live_gps_slot *slot = &live_stream_shared.gps_slots[slot_index];
        return live_send_slot_frame(server, LIVE_TX_DIAG_FRAME_GPS, LIVE_FRAME_GPS_BATCH, &slot->header, slot->payload);
    }

    return LIVE_SEND_SLOT_NONE;
}

static uint32_t live_send_ready_burst(struct tcpserver *server) {
    uint32_t sent_count = 0u;
    enum live_send_slot_result result;

    while (sent_count < LIVE_TX_MAX_BURST_FRAMES) {
        result = live_send_next_ready_slot(server);
        switch (result) {
            case LIVE_SEND_SLOT_SENT:
                sent_count++;
                break;
            case LIVE_SEND_SLOT_DROPPED:
            case LIVE_SEND_SLOT_BLOCKED:
            case LIVE_SEND_SLOT_NONE:
            default:
                return sent_count;
        }
    }

    return sent_count;
}

static void live_queue_start_request(struct tcpserver *server, const struct live_start_request_frame *frame) {
    if (server->live.phase != LIVE_PHASE_IDLE || live_stream_get_control_state() != LIVE_CONTROL_IDLE) {
        live_send_error(server, LIVE_START_RESULT_BUSY);
        return;
    }

    live_stream_shared.start_request = (struct live_start_request){
        .protocol_version = LIVE_PROTOCOL_VERSION,
        .requested_sensor_mask = frame->requested_sensor_mask,
        .requested_travel_hz = frame->travel_hz,
        .requested_imu_hz = frame->imu_hz,
        .requested_gps_fix_hz = frame->gps_fix_hz,
    };
    server->live.phase = LIVE_PHASE_STARTING;
    server->live.handshake_step = LIVE_HANDSHAKE_ACK;
    server->live.stop_queued = false;
    live_stream_set_control_state(LIVE_CONTROL_START_REQUESTED);
}

static void live_queue_stop_request(struct tcpserver *server) {
    switch (server->live.phase) {
        case LIVE_PHASE_STARTING:
        case LIVE_PHASE_START_HANDSHAKE:
            server->live.stop_queued = true;
            return;
        case LIVE_PHASE_ACTIVE:
            server->live.phase = LIVE_PHASE_STOPPING;
            live_stream_set_control_state(LIVE_CONTROL_STOP_REQUESTED);
            return;
        case LIVE_PHASE_IDLE:
            live_send_stop_ack(server);
            return;
        case LIVE_PHASE_STOPPING:
        case LIVE_PHASE_STOP_DEFERRED:
        default:
            return;
    }
}

static bool live_protocol_can_accept(const struct tcpserver *server) {
    return server->live.phase == LIVE_PHASE_IDLE && live_stream_get_control_state() == LIVE_CONTROL_IDLE &&
           !live_stream_shared.active;
}

static void live_protocol_on_accept(struct tcpserver *server) { live_reset_session(server); }

static void live_protocol_on_disconnect(struct tcpserver *server) { live_protocol_abort(server); }

static bool live_protocol_detect(const struct tcpserver *server) {
    uint32_t magic;

    if (server->rx_len < sizeof(magic)) {
        return false;
    }

    memcpy(&magic, server->rx_buffer, sizeof(magic));
    return magic == LIVE_PROTOCOL_MAGIC;
}

static bool live_protocol_process_rx(struct tcpserver *server) {
    while (server->rx_len >= sizeof(struct live_frame_header)) {
        struct live_frame_header header;
        uint32_t frame_bytes;
        uint8_t *payload;

        memcpy(&header, server->rx_buffer, sizeof(header));
        if (header.magic != LIVE_PROTOCOL_MAGIC || header.version != LIVE_PROTOCOL_VERSION) {
            LOG("LIVE", "RX rejected: bad magic/version (magic=0x%08x, ver=%u)\n", (unsigned)header.magic,
                (unsigned)header.version);
            return false;
        }

        if (header.payload_length > (TCPSERVER_RX_BUFFER_SIZE - sizeof(struct live_frame_header))) {
            LOG("LIVE", "RX rejected: payload too large (%u)\n", (unsigned)header.payload_length);
            return false;
        }

        frame_bytes = sizeof(struct live_frame_header) + header.payload_length;
        if (server->rx_len < frame_bytes) {
            return true;
        }

        payload = server->rx_buffer + sizeof(struct live_frame_header);
        switch (header.frame_type) {
            case LIVE_FRAME_START_LIVE:
                if (header.payload_length != sizeof(struct live_start_request_frame)) {
                    LOG("LIVE", "RX START_LIVE: bad payload size (%u, expected %u)\n", (unsigned)header.payload_length,
                        (unsigned)sizeof(struct live_start_request_frame));
                    live_send_error(server, LIVE_START_RESULT_INVALID_REQUEST);
                } else {
                    struct live_start_request_frame start_frame;
                    memcpy(&start_frame, payload, sizeof(start_frame));
                    live_queue_start_request(server, &start_frame);
                }
                break;
            case LIVE_FRAME_STOP_LIVE:
                if (header.payload_length != 0u) {
                    LOG("LIVE", "RX STOP_LIVE: unexpected payload size (%u)\n", (unsigned)header.payload_length);
                    live_send_error(server, LIVE_START_RESULT_INVALID_REQUEST);
                } else {
                    live_queue_stop_request(server);
                }
                break;
            case LIVE_FRAME_IDENTIFY: {
                if (header.payload_length != 0u) {
                    LOG("LIVE", "RX IDENTIFY: unexpected payload size (%u)\n", (unsigned)header.payload_length);
                    live_send_error(server, LIVE_START_RESULT_INVALID_REQUEST);
                    break;
                }

                pico_unique_board_id_t board_id;
                struct live_identify_ack_frame ack;
                pico_get_unique_board_id(&board_id);
                memcpy(ack.board_id, board_id.id, sizeof(ack.board_id));
                if (!live_send_frame(server, LIVE_TX_DIAG_FRAME_CONTROL, LIVE_FRAME_IDENTIFY_ACK, &ack, sizeof(ack),
                                     server->live.tx_sequence)) {
                    return true;
                }
                server->live.tx_sequence++;
                break;
            }
            case LIVE_FRAME_PING:
                if (header.payload_length != 0u) {
                    LOG("LIVE", "RX PING: unexpected payload size (%u)\n", (unsigned)header.payload_length);
                    live_send_error(server, LIVE_START_RESULT_INVALID_REQUEST);
                } else {
                    if (!live_send_frame(server, LIVE_TX_DIAG_FRAME_CONTROL, LIVE_FRAME_PONG, NULL, 0,
                                         server->live.tx_sequence)) {
                        return true;
                    }
                    server->live.tx_sequence++;
                }
                break;
            default:
                LOG("LIVE", "RX unknown frame type: %u\n", (unsigned)header.frame_type);
                live_send_error(server, LIVE_START_RESULT_INVALID_REQUEST);
                break;
        }

        tcpserver_consume_rx(server, (uint16_t)frame_bytes);
    }

    return true;
}

static void live_service_starting(struct tcpserver *server) {
    if (live_stream_get_control_state() != LIVE_CONTROL_START_RESPONSE_READY) {
        return;
    }

    server->live.phase = LIVE_PHASE_START_HANDSHAKE;
    server->live.handshake_step = LIVE_HANDSHAKE_ACK;
}

static void live_service_start_handshake(struct tcpserver *server) {
    switch (server->live.handshake_step) {
        case LIVE_HANDSHAKE_ACK:
            if (!live_send_start_ack(server)) {
                return;
            }
            if (live_stream_shared.start_response.result != LIVE_START_RESULT_OK) {
                if (server->live.stop_queued) {
                    server->live.phase = LIVE_PHASE_STOP_DEFERRED;
                } else {
                    server->live.phase = LIVE_PHASE_IDLE;
                }
                live_stream_set_control_state(LIVE_CONTROL_IDLE);
                return;
            }
            server->live.handshake_step = LIVE_HANDSHAKE_HEADER;
            return;
        case LIVE_HANDSHAKE_HEADER:
            if (!live_send_session_header(server)) {
                return;
            }
            server->live.handshake_step = LIVE_HANDSHAKE_STATS;
            return;
        case LIVE_HANDSHAKE_STATS:
            if (!live_send_session_stats(server)) {
                return;
            }
            server->live.handshake_step = LIVE_HANDSHAKE_DONE;
            return;
        case LIVE_HANDSHAKE_DONE:
            // STOP_LIVE arrived while we were sending ACK/HEADER/STATS — skip ACTIVE.
            if (server->live.stop_queued) {
                server->live.stop_queued = false;
                server->live.phase = LIVE_PHASE_STOPPING;
                live_stream_set_control_state(LIVE_CONTROL_STOP_REQUESTED);
                return;
            }
            server->live.phase = LIVE_PHASE_ACTIVE;
            server->live.last_stats_us = time_us_64();
            live_stream_set_control_state(LIVE_CONTROL_IDLE);
            return;
    }
}

static void live_service_stopping(struct tcpserver *server) {
    if (live_stream_get_control_state() != LIVE_CONTROL_STOP_RESPONSE_READY) {
        return;
    }

    if (server->client_pcb != NULL && !live_send_stop_ack(server)) {
        return;
    }
    live_log_tx_diag_summary(server, "stop");
    live_reset_tx_diag(server);
    server->live.phase = LIVE_PHASE_IDLE;
    live_stream_set_control_state(LIVE_CONTROL_IDLE);
}

static void live_service_stop_deferred(struct tcpserver *server) {
    if (live_stream_shared.active || live_stream_get_control_state() != LIVE_CONTROL_IDLE) {
        return;
    }

    if (server->client_pcb != NULL && !live_send_stop_ack(server)) {
        return;
    }
    live_log_tx_diag_summary(server, "defer");
    live_reset_tx_diag(server);
    server->live.phase = LIVE_PHASE_IDLE;
}

static void live_service_active(struct tcpserver *server) {
    uint64_t now_us;

    if (server->client_pcb == NULL) {
        return;
    }

    live_tx_diag_update_queue_state(server);
    live_send_ready_burst(server);
    live_tx_diag_update_queue_state(server);
    now_us = time_us_64();
    live_maybe_send_session_stats(server, now_us);

    if (live_tx_stall_timed_out(server)) {
        server->live.tx_timeout_fired = true;
        server->live.tx_timeout_fired_us = time_us_64();
        if (live_stream_shared.active && live_stream_get_control_state() == LIVE_CONTROL_IDLE) {
            live_stream_set_control_state(LIVE_CONTROL_STOP_REQUESTED);
            server->live.tx_timeout_requested_stop = true;
        }

        live_watchdog_diag_log_snapshot("tx_timeout");
        live_watchdog_diag_session_stop();
        server->last_error = PICO_ERROR_TIMEOUT;
        server->live.tx_timeout_requested_close = true;
        live_log_tx_diag_summary(server, "timeout");
        server->close_client_requested = true;
    }
}

static bool live_protocol_service(struct tcpserver *server) {
    switch (server->live.phase) {
        case LIVE_PHASE_STARTING:
            live_watchdog_diag_mark_core1(LIVE_WATCHDOG_CORE1_MARKER_SERVICE_STARTING);
            live_service_starting(server);
            break;
        case LIVE_PHASE_START_HANDSHAKE:
            live_watchdog_diag_mark_core1(LIVE_WATCHDOG_CORE1_MARKER_SERVICE_HANDSHAKE);
            live_service_start_handshake(server);
            break;
        case LIVE_PHASE_ACTIVE:
            live_watchdog_diag_mark_core1(LIVE_WATCHDOG_CORE1_MARKER_SERVICE_ACTIVE);
            live_service_active(server);
            break;
        case LIVE_PHASE_STOPPING:
            live_watchdog_diag_mark_core1(LIVE_WATCHDOG_CORE1_MARKER_SERVICE_STOPPING);
            live_service_stopping(server);
            break;
        case LIVE_PHASE_STOP_DEFERRED:
            live_watchdog_diag_mark_core1(LIVE_WATCHDOG_CORE1_MARKER_SERVICE_STOP_DEFERRED);
            live_service_stop_deferred(server);
            break;
        default:
            break;
    }

    return false;
}

static bool live_protocol_needs_service(const struct tcpserver *server) {
    return server->live.phase != LIVE_PHASE_IDLE;
}

static void live_protocol_wait_for_quiescent_control(struct tcpserver *server) {
    enum live_control_state cs;

    (void)server;

    // Disconnect cleanup must not return until the shared live control state is quiescent.
    // tcpserver_reset_connection_state() wipes the local session flags immediately after.
    for (;;) {
        live_watchdog_diag_mark_core1(LIVE_WATCHDOG_CORE1_MARKER_WAIT_QUIESCENT);

        cs = live_stream_get_control_state();

        if (cs == LIVE_CONTROL_STOP_RESPONSE_READY) {
            live_stream_set_control_state(LIVE_CONTROL_IDLE);
            break;
        }
        if (cs == LIVE_CONTROL_START_RESPONSE_READY) {
            // Core 0 finished a start we tried to override with STOP_REQUESTED.
            // Acknowledge it, then re-request stop if a session was actually started.
            live_stream_set_control_state(LIVE_CONTROL_IDLE);
            if (live_stream_shared.active) {
                live_stream_set_control_state(LIVE_CONTROL_STOP_REQUESTED);
                continue;
            }
            break;
        }
        if (cs == LIVE_CONTROL_IDLE) {
            if (!live_stream_shared.active) {
                break;
            }
            // Session is running but no stop was requested yet (e.g. disconnect mid-handshake).
            live_stream_set_control_state(LIVE_CONTROL_STOP_REQUESTED);
        } else if (cs == LIVE_CONTROL_START_REQUESTED) {
            // Core 0 hasn't seen the start yet — overwrite with stop.
            live_stream_set_control_state(LIVE_CONTROL_STOP_REQUESTED);
        }

        tight_loop_contents();
    }
}

// Called from on_disconnect, always on Core 1 (never from tcp_server_err ISR —
// that path defers via err_disconnect_pending). Safe to spin-wait on Core 0.
static void live_protocol_abort(struct tcpserver *server) {
    enum live_control_state cs = live_stream_get_control_state();

    live_watchdog_diag_mark_core1(LIVE_WATCHDOG_CORE1_MARKER_ABORT);

    if (server->live.phase == LIVE_PHASE_IDLE && !live_stream_shared.active && cs == LIVE_CONTROL_IDLE) {
        return;
    }

    server->live.phase = LIVE_PHASE_STOPPING;
    server->live.stop_queued = false;
    live_watchdog_diag_log_snapshot("abort");
    live_log_tx_diag_summary(server, "abort");
    live_protocol_wait_for_quiescent_control(server);
    live_reset_tx_diag(server);
}
