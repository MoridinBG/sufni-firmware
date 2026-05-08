#ifndef UBX_H
#define UBX_H

#include "gps_sensor.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// UBX payload and message IDs used by the generic transport and M8N policy.
#define UBX_MAX_PAYLOAD 512

#define UBX_CLASS_NAV  0x01
#define UBX_CLASS_RXM  0x02
#define UBX_CLASS_ACK  0x05
#define UBX_CLASS_CFG  0x06
#define UBX_CLASS_NMEA 0xF0

#define UBX_ID_ACK_NACK  0x00
#define UBX_ID_ACK_ACK   0x01
#define UBX_ID_CFG_PRT   0x00
#define UBX_ID_CFG_MSG   0x01
#define UBX_ID_CFG_RST   0x04
#define UBX_ID_CFG_RATE  0x08
#define UBX_ID_CFG_CFG   0x09
#define UBX_ID_CFG_NAV5  0x24
#define UBX_ID_CFG_GNSS  0x3E
#define UBX_ID_NAV_PVT   0x07
#define UBX_ID_RXM_PMREQ 0x41

// Called for each checksum-valid UBX frame delivered by the transport.
typedef void (*ubx_frame_handler_t)(void *context, uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len);

// Called by blocking ACK/poll helpers so the owner can drain RX bytes.
typedef void (*ubx_service_fn)(void *context);

// Outcome of a UBX command transaction that expects ACK-ACK or ACK-NAK.
enum ubx_ack_result {
    UBX_ACK_TIMEOUT = -1,
    UBX_ACK_NACK = 0,
    UBX_ACK_ACK = 1,
};

// Byte-parser states for the UBX frame machine.
enum ubx_parse_state {
    UBX_PARSE_IDLE,
    UBX_PARSE_SYNC2,
    UBX_PARSE_HEADER,
    UBX_PARSE_PAYLOAD,
    UBX_PARSE_CK_A,
    UBX_PARSE_CK_B,
};

// Parser and transaction state for one UBX byte stream.
// The owner provides storage so the transport does not rely on file-level globals.
struct ubx_transport {
    enum ubx_parse_state parse_state;
    uint8_t header[4];
    size_t header_pos;
    uint8_t payload[UBX_MAX_PAYLOAD];
    uint16_t payload_len;
    uint16_t payload_pos;
    uint8_t ck_a;
    uint8_t ck_b;
    uint8_t received_ck_a;

    bool waiting_for_ack;
    uint8_t expected_ack_cls;
    uint8_t expected_ack_id;
    enum ubx_ack_result ack_result;

    bool waiting_for_poll;
    uint8_t expected_poll_cls;
    uint8_t expected_poll_id;
    uint8_t *poll_buf;
    uint16_t poll_cap;
    int poll_payload_len;

    ubx_frame_handler_t frame_handler;
    void *frame_context;
};

// Initializes parser and transaction state, preserving no previous wait state.
void ubx_transport_init(struct ubx_transport *transport, ubx_frame_handler_t handler, void *context);

// Resets only frame parsing state; pending ACK/poll waits are left untouched.
void ubx_transport_reset_parser(struct ubx_transport *transport);

// Clears pending ACK and poll transactions without changing parser state.
void ubx_transport_clear_wait_state(struct ubx_transport *transport);

// Feeds one byte into the UBX parser and dispatches complete checksum-valid frames.
void ubx_transport_process_byte(struct ubx_transport *transport, uint8_t byte);

// Sends one framed UBX message on the GPS UART.
void ubx_send(struct gps_sensor *gps, uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len);

// Sends a UBX command and services the receiver until ACK, NACK, or timeout.
enum ubx_ack_result ubx_send_ack_wait(struct ubx_transport *transport, struct gps_sensor *gps, uint8_t cls, uint8_t id,
                                      const uint8_t *payload, uint16_t len, ubx_service_fn service, void *context,
                                      uint32_t timeout_ms);

// Sends a UBX poll request and copies the matching response payload into out_buf.
int ubx_poll_response(struct ubx_transport *transport, struct gps_sensor *gps, uint8_t cls, uint8_t id,
                      const uint8_t *req_payload, uint16_t req_len, uint8_t *out_buf, uint16_t out_cap,
                      ubx_service_fn service, void *context, uint32_t timeout_ms);

// Reads little-endian integer values from UBX payloads.
uint16_t ubx_read_u16(const uint8_t *payload, uint16_t offset);
uint32_t ubx_read_u32(const uint8_t *payload, uint16_t offset);
int32_t ubx_read_i32(const uint8_t *payload, uint16_t offset);

// Writes little-endian integer values into UBX payloads.
void ubx_write_u16(uint8_t *payload, uint16_t offset, uint16_t value);
void ubx_write_u32(uint8_t *payload, uint16_t offset, uint32_t value);

#endif
