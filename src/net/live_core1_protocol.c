#include "live_core1_protocol.h"

#include "live_protocol.h"
#include "tcpserver.h"

#include "../fw/live_stream_shared.h"
#include "../fw/sensor_setup.h"
#include "../util/log.h"

#include "lwip/tcp.h"
#include "pico/cyw43_arch.h"
#include "pico/time.h"
#include "pico/unique_id.h"

#include <string.h>

#define LIVE_TX_FRAME_BUFFER_SIZE                                                                                      \
    (sizeof(struct live_frame_header) + sizeof(struct live_batch_payload) +                                            \
     (LIVE_IMU_RECORDS_PER_SLOT * sizeof(struct imu_record)))

static uint8_t live_tx_frame_buffer[LIVE_TX_FRAME_BUFFER_SIZE];

static bool live_protocol_can_accept(const struct tcpserver *server);
static void live_protocol_on_accept(struct tcpserver *server);
static void live_protocol_on_disconnect(struct tcpserver *server);
static bool live_protocol_detect(const struct tcpserver *server);
static bool live_protocol_process_rx(struct tcpserver *server);
static void live_protocol_service(struct tcpserver *server);
static bool live_protocol_needs_service(const struct tcpserver *server);
static void live_protocol_wait_for_quiescent_control(struct tcpserver *server);
static void live_protocol_abort(struct tcpserver *server);

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
    server->live = (struct live_protocol_session){
        .tx_sequence = 1,
        .last_stats_us = time_us_64(),
    };
}

static bool live_write_frame_bytes(struct tcpserver *server, const void *frame_bytes, uint32_t total_bytes) {
    err_t err;

    if (server->client_pcb == NULL) {
        return false;
    }

    cyw43_arch_lwip_begin();
    if (tcp_sndbuf(server->client_pcb) < total_bytes) {
        cyw43_arch_lwip_end();
        return false;
    }

    err = tcp_write(server->client_pcb, frame_bytes, total_bytes, TCP_WRITE_FLAG_COPY);
    if (err == ERR_OK) {
        tcp_output(server->client_pcb);
    }
    cyw43_arch_lwip_end();

    return err == ERR_OK;
}

static bool live_send_frame(struct tcpserver *server, uint16_t frame_type, const void *payload, uint32_t payload_length,
                            uint32_t sequence) {
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

    return live_write_frame_bytes(server, live_tx_frame_buffer, total_bytes);
}

static bool live_send_error(struct tcpserver *server, int32_t error_code) {
    uint32_t sequence = server->live.tx_sequence;
    struct live_error_frame frame = {.error_code = error_code};

    LOG("LIVE", "Sending error: %d\n", (int)error_code);
    if (!live_send_frame(server, LIVE_FRAME_ERROR, &frame, sizeof(frame), sequence)) {
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

    LOG("LIVE", "Sending START_ACK: result=%d, session=%u, mask=0x%02x\n", (int)frame.result,
        (unsigned)frame.session_id, (unsigned)frame.selected_sensor_mask);
    if (!live_send_frame(server, LIVE_FRAME_START_LIVE_ACK, &frame, sizeof(frame), sequence)) {
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

    if (!live_send_frame(server, LIVE_FRAME_STOP_LIVE_ACK, &frame, sizeof(frame), sequence)) {
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

    if (!live_send_frame(server, LIVE_FRAME_SESSION_HEADER, &frame, sizeof(frame), sequence)) {
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

    if (!live_send_frame(server, LIVE_FRAME_SESSION_STATS, &frame, sizeof(frame), sequence)) {
        return false;
    }

    server->live.tx_sequence++;
    return true;
}

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

static bool live_send_slot_frame(struct tcpserver *server, uint16_t frame_type, struct live_slot_header *header,
                                 const void *payload) {
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
    if (total_bytes > sizeof(live_tx_frame_buffer)) {
        __atomic_store_n(&header->state, LIVE_SLOT_READY, __ATOMIC_RELEASE);
        return false;
    }

    memcpy(live_tx_frame_buffer, &frame_header, sizeof(frame_header));
    memcpy(live_tx_frame_buffer + sizeof(frame_header), &batch, sizeof(batch));
    if (header->payload_bytes > 0u) {
        memcpy(live_tx_frame_buffer + sizeof(frame_header) + sizeof(batch), payload, header->payload_bytes);
    }

    if (!live_write_frame_bytes(server, live_tx_frame_buffer, total_bytes)) {
        __atomic_store_n(&header->state, LIVE_SLOT_READY, __ATOMIC_RELEASE);
        return false;
    }

    server->live.tx_sequence++;
    live_stream_release_slot(&header->state);
    return true;
}

static bool live_send_next_ready_slot(struct tcpserver *server) {
    int slot_index;

    slot_index = live_claim_oldest_ready_slot(live_stream_shared.travel_slots, sizeof(struct live_travel_slot),
                                              LIVE_TRAVEL_SLOT_COUNT);
    if (slot_index >= 0) {
        struct live_travel_slot *slot = &live_stream_shared.travel_slots[slot_index];
        return live_send_slot_frame(server, LIVE_FRAME_TRAVEL_BATCH, &slot->header, slot->payload);
    }

    slot_index =
        live_claim_oldest_ready_slot(live_stream_shared.imu_slots, sizeof(struct live_imu_slot), LIVE_IMU_SLOT_COUNT);
    if (slot_index >= 0) {
        struct live_imu_slot *slot = &live_stream_shared.imu_slots[slot_index];
        return live_send_slot_frame(server, LIVE_FRAME_IMU_BATCH, &slot->header, slot->payload);
    }

    slot_index =
        live_claim_oldest_ready_slot(live_stream_shared.gps_slots, sizeof(struct live_gps_slot), LIVE_GPS_SLOT_COUNT);
    if (slot_index >= 0) {
        struct live_gps_slot *slot = &live_stream_shared.gps_slots[slot_index];
        return live_send_slot_frame(server, LIVE_FRAME_GPS_BATCH, &slot->header, slot->payload);
    }

    return false;
}

static void live_queue_start_request(struct tcpserver *server, const struct live_start_request_frame *frame) {
    if (server->live.phase != LIVE_PHASE_IDLE || live_stream_get_control_state() != LIVE_CONTROL_IDLE) {
        live_send_error(server, LIVE_START_RESULT_BUSY);
        return;
    }

    live_stream_shared.start_request = (struct live_start_request){
        .protocol_version = LIVE_PROTOCOL_VERSION,
        .sensor_mask = frame->sensor_mask,
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
                if (!live_send_frame(server, LIVE_FRAME_IDENTIFY_ACK, &ack, sizeof(ack), server->live.tx_sequence)) {
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
                    if (!live_send_frame(server, LIVE_FRAME_PONG, NULL, 0, server->live.tx_sequence)) {
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
            if (server->live.stop_queued) {
                server->live.stop_queued = false;
                server->live.phase = LIVE_PHASE_STOPPING;
                live_stream_set_control_state(LIVE_CONTROL_STOP_REQUESTED);
                return;
            }
            LOG("LIVE", "Session %u started, mask=0x%02x\n", (unsigned)live_stream_shared.start_response.session_id,
                (unsigned)live_stream_shared.start_response.selected_sensor_mask);
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

    LOG("LIVE", "Session stopped\n");
    if (server->client_pcb != NULL && !live_send_stop_ack(server)) {
        return;
    }
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
    server->live.phase = LIVE_PHASE_IDLE;
}

static void live_service_active(struct tcpserver *server) {
    if (server->client_pcb == NULL) {
        return;
    }

    live_send_next_ready_slot(server);
    if ((time_us_64() - server->live.last_stats_us) >= 500000u) {
        if (live_send_session_stats(server)) {
            server->live.last_stats_us = time_us_64();
        }
    }
}

static void live_protocol_service(struct tcpserver *server) {
    switch (server->live.phase) {
        case LIVE_PHASE_STARTING:
            live_service_starting(server);
            break;
        case LIVE_PHASE_START_HANDSHAKE:
            live_service_start_handshake(server);
            break;
        case LIVE_PHASE_ACTIVE:
            live_service_active(server);
            break;
        case LIVE_PHASE_STOPPING:
            live_service_stopping(server);
            break;
        case LIVE_PHASE_STOP_DEFERRED:
            live_service_stop_deferred(server);
            break;
        default:
            break;
    }
}

static bool live_protocol_needs_service(const struct tcpserver *server) {
    return server->live.phase != LIVE_PHASE_IDLE;
}

static void live_protocol_wait_for_quiescent_control(struct tcpserver *server) {
    enum live_control_state cs;

    // Disconnect cleanup must not return until the shared live control state is quiescent.
    // tcpserver_reset_connection_state() wipes the local session flags immediately after.
    for (;;) {
        cs = live_stream_get_control_state();

        if (cs == LIVE_CONTROL_STOP_RESPONSE_READY) {
            live_stream_set_control_state(LIVE_CONTROL_IDLE);
            break;
        }
        if (cs == LIVE_CONTROL_START_RESPONSE_READY) {
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
            live_stream_set_control_state(LIVE_CONTROL_STOP_REQUESTED);
        } else if (cs == LIVE_CONTROL_START_REQUESTED) {
            live_stream_set_control_state(LIVE_CONTROL_STOP_REQUESTED);
        }

        tight_loop_contents();
    }
}

static void live_protocol_abort(struct tcpserver *server) {
    enum live_control_state cs = live_stream_get_control_state();

    if (server->live.phase == LIVE_PHASE_IDLE && !live_stream_shared.active && cs == LIVE_CONTROL_IDLE) {
        return;
    }

    LOG("LIVE", "Aborting session (phase=%d, stop_queued=%d)\n", (int)server->live.phase, server->live.stop_queued);

    server->live.phase = LIVE_PHASE_STOPPING;
    server->live.stop_queued = false;
    live_protocol_wait_for_quiescent_control(server);

    LOG("LIVE", "Abort complete\n");
}
