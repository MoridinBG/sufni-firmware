#include "live_stream_core1.h"

#include "live_protocol.h"
#include "tcpserver.h"

#include "../fw/live_stream_shared.h"
#include "../fw/sensor_setup.h"

#include "lwip/tcp.h"
#include "pico/cyw43_arch.h"
#include "pico/time.h"

#include <string.h>

#include "../util/log.h"

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
    err_t err = ERR_OK;

    cyw43_arch_lwip_begin();
    if (tcp_sndbuf(server->client_pcb) < total_bytes) {
        cyw43_arch_lwip_end();
        return false;
    }
    err = tcp_write(server->client_pcb, &header, sizeof(header), TCP_WRITE_FLAG_COPY | TCP_WRITE_FLAG_MORE);
    if (err == ERR_OK && payload_length > 0u) {
        err = tcp_write(server->client_pcb, payload, payload_length, TCP_WRITE_FLAG_COPY);
    }
    if (err == ERR_OK) {
        tcp_output(server->client_pcb);
    }
    cyw43_arch_lwip_end();

    return err == ERR_OK;
}

static void live_consume_rx(struct tcpserver *server, uint16_t bytes_to_consume) {
    if (bytes_to_consume >= server->rx_len) {
        server->rx_len = 0;
        return;
    }

    memmove(server->rx_buffer, server->rx_buffer + bytes_to_consume, server->rx_len - bytes_to_consume);
    server->rx_len -= bytes_to_consume;
}

static bool live_send_error(struct tcpserver *server, int32_t error_code) {
    LOG("LIVE", "Sending error: %d\n", (int)error_code);
    struct live_error_frame frame = {.error_code = error_code};
    return live_send_frame(server, LIVE_FRAME_ERROR, &frame, sizeof(frame), server->live_tx_sequence++);
}

static bool live_send_start_ack(struct tcpserver *server) {
    struct live_start_ack_frame frame = {
        .result = live_stream_shared.start_response.result,
        .session_id = live_stream_shared.start_response.session_id,
        .selected_sensor_mask = live_stream_shared.start_response.selected_sensor_mask,
    };

    LOG("LIVE", "Sending START_ACK: result=%d, session=%u, mask=0x%02x\n", (int)frame.result,
        (unsigned)frame.session_id, (unsigned)frame.selected_sensor_mask);
    return live_send_frame(server, LIVE_FRAME_START_LIVE_ACK, &frame, sizeof(frame), server->live_tx_sequence++);
}

static bool live_send_stop_ack(struct tcpserver *server) {
    struct live_stop_ack_frame frame = {
        .session_id = live_stream_shared.start_response.session_id,
    };

    return live_send_frame(server, LIVE_FRAME_STOP_LIVE_ACK, &frame, sizeof(frame), server->live_tx_sequence++);
}

static bool live_send_session_header(struct tcpserver *server) {
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

    return live_send_frame(server, LIVE_FRAME_SESSION_HEADER, &frame, sizeof(frame), server->live_tx_sequence++);
}

static bool live_send_session_stats(struct tcpserver *server) {
    struct live_session_stats_frame frame = {
        .session_id = live_stream_shared.start_response.session_id,
        .travel_queue_depth = live_stream_shared.travel_stats.queue_depth,
        .imu_queue_depth = live_stream_shared.imu_stats.queue_depth,
        .gps_queue_depth = live_stream_shared.gps_stats.queue_depth,
        .travel_dropped_batches = live_stream_shared.travel_stats.dropped_batches,
        .imu_dropped_batches = live_stream_shared.imu_stats.dropped_batches,
        .gps_dropped_batches = live_stream_shared.gps_stats.dropped_batches,
    };

    return live_send_frame(server, LIVE_FRAME_SESSION_STATS, &frame, sizeof(frame), server->live_tx_sequence++);
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
        .sequence = server->live_tx_sequence,
    };
    uint32_t total_bytes = sizeof(frame_header) + sizeof(batch) + header->payload_bytes;
    err_t err = ERR_OK;

    cyw43_arch_lwip_begin();
    if (tcp_sndbuf(server->client_pcb) < total_bytes) {
        cyw43_arch_lwip_end();
        __atomic_store_n(&header->state, LIVE_SLOT_READY, __ATOMIC_RELEASE);
        return false;
    }
    err = tcp_write(server->client_pcb, &frame_header, sizeof(frame_header), TCP_WRITE_FLAG_COPY | TCP_WRITE_FLAG_MORE);
    if (err == ERR_OK) {
        err = tcp_write(server->client_pcb, &batch, sizeof(batch), TCP_WRITE_FLAG_COPY | TCP_WRITE_FLAG_MORE);
    }
    if (err == ERR_OK && header->payload_bytes > 0u) {
        err = tcp_write(server->client_pcb, payload, header->payload_bytes, TCP_WRITE_FLAG_COPY);
    }
    if (err == ERR_OK) {
        server->live_tx_sequence++;
        tcp_output(server->client_pcb);
    }
    cyw43_arch_lwip_end();

    if (err != ERR_OK) {
        __atomic_store_n(&header->state, LIVE_SLOT_READY, __ATOMIC_RELEASE);
        return false;
    }

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
    if (server->live_start_pending || server->live_session_active ||
        live_stream_get_control_state() != LIVE_CONTROL_IDLE) {
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
    server->live_start_pending = true;
    live_stream_set_control_state(LIVE_CONTROL_START_REQUESTED);
}

static void live_queue_stop_request(struct tcpserver *server) {
    if (!server->live_session_active) {
        live_send_stop_ack(server);
        return;
    }

    if (!server->live_stop_pending) {
        server->live_stop_pending = true;
        live_stream_set_control_state(LIVE_CONTROL_STOP_REQUESTED);
    }
}

bool live_stream_core1_can_accept_client(const struct tcpserver *server) {
    return live_stream_get_control_state() == LIVE_CONTROL_IDLE && !live_stream_shared.active &&
           !server->live_session_active && !server->live_start_pending && !server->live_stop_pending;
}

void live_stream_core1_reset(struct tcpserver *server) {
    server->protocol_mode = TCPSERVER_PROTOCOL_UNKNOWN;
    server->live_session_active = false;
    server->live_start_pending = false;
    server->live_stop_pending = false;
    server->live_tx_sequence = 1;
    server->last_stats_us = time_us_64();
    server->rx_len = 0;
    memset(server->rx_buffer, 0, sizeof(server->rx_buffer));
}

bool live_stream_core1_process_rx(struct tcpserver *server) {
    while (server->rx_len >= sizeof(struct live_frame_header)) {
        struct live_frame_header header;
        uint16_t frame_bytes;
        uint8_t *payload;

        memcpy(&header, server->rx_buffer, sizeof(header));
        if (header.magic != LIVE_PROTOCOL_MAGIC || header.version != LIVE_PROTOCOL_VERSION) {
            LOG("LIVE", "RX rejected: bad magic/version (magic=0x%08x, ver=%u)\n", (unsigned)header.magic,
                (unsigned)header.version);
            return false;
        }

        frame_bytes = (uint16_t)(sizeof(struct live_frame_header) + header.payload_length);
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
                live_queue_stop_request(server);
                break;
            case LIVE_FRAME_PING:
                live_send_frame(server, LIVE_FRAME_PONG, NULL, 0, server->live_tx_sequence++);
                break;
            default:
                LOG("LIVE", "RX unknown frame type: %u\n", (unsigned)header.frame_type);
                live_send_error(server, LIVE_START_RESULT_INVALID_REQUEST);
                break;
        }

        live_consume_rx(server, frame_bytes);
    }

    return true;
}

void live_stream_core1_service(struct tcpserver *server) {
    if (server->live_start_pending && live_stream_get_control_state() == LIVE_CONTROL_START_RESPONSE_READY) {
        live_send_start_ack(server);
        if (live_stream_shared.start_response.result == LIVE_START_RESULT_OK) {
            LOG("LIVE", "Session %u started, mask=0x%02x\n", (unsigned)live_stream_shared.start_response.session_id,
                (unsigned)live_stream_shared.start_response.selected_sensor_mask);
            server->live_session_active = true;
            live_send_session_header(server);
            live_send_session_stats(server);
        }
        server->live_start_pending = false;
        live_stream_set_control_state(LIVE_CONTROL_IDLE);
    }

    if (server->live_stop_pending && live_stream_get_control_state() == LIVE_CONTROL_STOP_RESPONSE_READY) {
        LOG("LIVE", "Session stopped\n");
        if (server->client_pcb != NULL) {
            live_send_stop_ack(server);
        }
        server->live_start_pending = false;
        server->live_session_active = false;
        server->live_stop_pending = false;
        live_stream_set_control_state(LIVE_CONTROL_IDLE);
    }

    if (server->live_session_active && server->client_pcb != NULL) {
        live_send_next_ready_slot(server);
        if ((time_us_64() - server->last_stats_us) >= 500000u) {
            live_send_session_stats(server);
            server->last_stats_us = time_us_64();
        }
    }
}

void live_stream_core1_abort(struct tcpserver *server) {
    if ((server->live_start_pending || server->live_session_active || server->live_stop_pending ||
         live_stream_shared.active || live_stream_get_control_state() != LIVE_CONTROL_IDLE) &&
        !server->live_stop_pending) {
        LOG("LIVE", "Aborting session (active=%d, start_pending=%d, stop_pending=%d)\n", server->live_session_active,
            server->live_start_pending, server->live_stop_pending);
        server->live_start_pending = false;
        server->live_session_active = live_stream_shared.active;
        server->live_stop_pending = true;
        live_stream_set_control_state(LIVE_CONTROL_STOP_REQUESTED);
    }
}