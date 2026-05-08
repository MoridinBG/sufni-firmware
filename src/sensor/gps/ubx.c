#include "ubx.h"
#include "hardware/uart.h"
#include "pico/time.h"
#include <string.h>

#define UBX_SYNC_1 0xB5
#define UBX_SYNC_2 0x62

static void ubx_checksum_byte(uint8_t byte, uint8_t *ck_a, uint8_t *ck_b) {
    *ck_a = (uint8_t)(*ck_a + byte);
    *ck_b = (uint8_t)(*ck_b + *ck_a);
}

void ubx_transport_init(struct ubx_transport *transport, ubx_frame_handler_t handler, void *context) {
    memset(transport, 0, sizeof(*transport));
    transport->parse_state = UBX_PARSE_IDLE;
    transport->ack_result = UBX_ACK_TIMEOUT;
    transport->poll_payload_len = -1;
    transport->frame_handler = handler;
    transport->frame_context = context;
}

void ubx_transport_reset_parser(struct ubx_transport *transport) {
    transport->parse_state = UBX_PARSE_IDLE;
    transport->header_pos = 0;
    transport->payload_len = 0;
    transport->payload_pos = 0;
    transport->ck_a = 0;
    transport->ck_b = 0;
    transport->received_ck_a = 0;
}

void ubx_transport_clear_wait_state(struct ubx_transport *transport) {
    transport->waiting_for_ack = false;
    transport->expected_ack_cls = 0;
    transport->expected_ack_id = 0;
    transport->ack_result = UBX_ACK_TIMEOUT;

    transport->waiting_for_poll = false;
    transport->expected_poll_cls = 0;
    transport->expected_poll_id = 0;
    transport->poll_buf = NULL;
    transport->poll_cap = 0;
    transport->poll_payload_len = -1;
}

static void ubx_on_frame(struct ubx_transport *transport, uint8_t cls, uint8_t id, const uint8_t *payload,
                         uint16_t len) {
    if (cls == UBX_CLASS_ACK && (id == UBX_ID_ACK_ACK || id == UBX_ID_ACK_NACK) && len == 2) {
        if (transport->waiting_for_ack && payload[0] == transport->expected_ack_cls &&
            payload[1] == transport->expected_ack_id) {
            transport->ack_result = (id == UBX_ID_ACK_ACK) ? UBX_ACK_ACK : UBX_ACK_NACK;
            transport->waiting_for_ack = false;
        }
        if (transport->frame_handler != NULL) {
            transport->frame_handler(transport->frame_context, cls, id, payload, len);
        }
        return;
    }

    if (transport->waiting_for_poll && cls == transport->expected_poll_cls && id == transport->expected_poll_id) {
        uint16_t copy_len = len < transport->poll_cap ? len : transport->poll_cap;
        if (copy_len > 0 && transport->poll_buf != NULL) {
            memcpy(transport->poll_buf, payload, copy_len);
        }
        transport->poll_payload_len = len;
        transport->waiting_for_poll = false;
    }

    if (transport->frame_handler != NULL) {
        transport->frame_handler(transport->frame_context, cls, id, payload, len);
    }
}

void ubx_transport_process_byte(struct ubx_transport *transport, uint8_t byte) {
    switch (transport->parse_state) {
        case UBX_PARSE_IDLE:
            if (byte == UBX_SYNC_1) {
                transport->parse_state = UBX_PARSE_SYNC2;
            }
            break;

        case UBX_PARSE_SYNC2:
            if (byte == UBX_SYNC_2) {
                transport->header_pos = 0;
                transport->payload_len = 0;
                transport->payload_pos = 0;
                transport->ck_a = 0;
                transport->ck_b = 0;
                transport->parse_state = UBX_PARSE_HEADER;
            } else {
                transport->parse_state = UBX_PARSE_IDLE;
            }
            break;

        case UBX_PARSE_HEADER:
            transport->header[transport->header_pos++] = byte;
            ubx_checksum_byte(byte, &transport->ck_a, &transport->ck_b);
            if (transport->header_pos == sizeof(transport->header)) {
                transport->payload_len = (uint16_t)transport->header[2] | ((uint16_t)transport->header[3] << 8);
                if (transport->payload_len > sizeof(transport->payload)) {
                    ubx_transport_reset_parser(transport);
                } else if (transport->payload_len == 0) {
                    transport->parse_state = UBX_PARSE_CK_A;
                } else {
                    transport->parse_state = UBX_PARSE_PAYLOAD;
                }
            }
            break;

        case UBX_PARSE_PAYLOAD:
            transport->payload[transport->payload_pos++] = byte;
            ubx_checksum_byte(byte, &transport->ck_a, &transport->ck_b);
            if (transport->payload_pos == transport->payload_len) {
                transport->parse_state = UBX_PARSE_CK_A;
            }
            break;

        case UBX_PARSE_CK_A:
            transport->received_ck_a = byte;
            transport->parse_state = UBX_PARSE_CK_B;
            break;

        case UBX_PARSE_CK_B:
            if (transport->received_ck_a == transport->ck_a && byte == transport->ck_b) {
                ubx_on_frame(transport, transport->header[0], transport->header[1], transport->payload,
                             transport->payload_len);
            }
            ubx_transport_reset_parser(transport);
            break;
    }
}

void ubx_send(struct gps_sensor *gps, uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len) {
    uint8_t header[6] = {UBX_SYNC_1, UBX_SYNC_2, cls, id, (uint8_t)(len & 0xFF), (uint8_t)(len >> 8)};
    uint8_t ck_a = 0;
    uint8_t ck_b = 0;

    for (size_t i = 2; i < sizeof(header); i++) { ubx_checksum_byte(header[i], &ck_a, &ck_b); }
    for (uint16_t i = 0; i < len; i++) { ubx_checksum_byte(payload[i], &ck_a, &ck_b); }

    uart_write_blocking(gps->comm.uart.instance, header, sizeof(header));
    if (len > 0) {
        uart_write_blocking(gps->comm.uart.instance, payload, len);
    }
    uint8_t checksum[2] = {ck_a, ck_b};
    uart_write_blocking(gps->comm.uart.instance, checksum, sizeof(checksum));
}

enum ubx_ack_result ubx_send_ack_wait(struct ubx_transport *transport, struct gps_sensor *gps, uint8_t cls, uint8_t id,
                                      const uint8_t *payload, uint16_t len, ubx_service_fn service, void *context,
                                      uint32_t timeout_ms) {
    service(context);
    transport->expected_ack_cls = cls;
    transport->expected_ack_id = id;
    transport->ack_result = UBX_ACK_TIMEOUT;
    transport->waiting_for_ack = true;

    ubx_send(gps, cls, id, payload, len);

    absolute_time_t deadline = make_timeout_time_ms(timeout_ms);
    while (transport->waiting_for_ack && !time_reached(deadline)) {
        service(context);
        tight_loop_contents();
    }

    enum ubx_ack_result result = transport->ack_result;
    transport->waiting_for_ack = false;
    return result;
}

int ubx_poll_response(struct ubx_transport *transport, struct gps_sensor *gps, uint8_t cls, uint8_t id,
                      const uint8_t *req_payload, uint16_t req_len, uint8_t *out_buf, uint16_t out_cap,
                      ubx_service_fn service, void *context, uint32_t timeout_ms) {
    service(context);
    transport->expected_poll_cls = cls;
    transport->expected_poll_id = id;
    transport->poll_buf = out_buf;
    transport->poll_cap = out_cap;
    transport->poll_payload_len = -1;
    transport->waiting_for_poll = true;

    ubx_send(gps, cls, id, req_payload, req_len);

    absolute_time_t deadline = make_timeout_time_ms(timeout_ms);
    while (transport->waiting_for_poll && !time_reached(deadline)) {
        service(context);
        tight_loop_contents();
    }

    transport->waiting_for_poll = false;
    if (transport->poll_payload_len < 0) {
        return -1;
    }

    int payload_len = transport->poll_payload_len;
    transport->poll_buf = NULL;
    transport->poll_cap = 0;
    return payload_len;
}

uint16_t ubx_read_u16(const uint8_t *payload, uint16_t offset) {
    return (uint16_t)payload[offset] | ((uint16_t)payload[offset + 1] << 8);
}

uint32_t ubx_read_u32(const uint8_t *payload, uint16_t offset) {
    return (uint32_t)payload[offset] | ((uint32_t)payload[offset + 1] << 8) | ((uint32_t)payload[offset + 2] << 16) |
           ((uint32_t)payload[offset + 3] << 24);
}

int32_t ubx_read_i32(const uint8_t *payload, uint16_t offset) { return (int32_t)ubx_read_u32(payload, offset); }

void ubx_write_u16(uint8_t *payload, uint16_t offset, uint16_t value) {
    payload[offset] = (uint8_t)(value & 0xFF);
    payload[offset + 1] = (uint8_t)(value >> 8);
}

void ubx_write_u32(uint8_t *payload, uint16_t offset, uint32_t value) {
    payload[offset] = (uint8_t)(value & 0xFF);
    payload[offset + 1] = (uint8_t)((value >> 8) & 0xFF);
    payload[offset + 2] = (uint8_t)((value >> 16) & 0xFF);
    payload[offset + 3] = (uint8_t)(value >> 24);
}
