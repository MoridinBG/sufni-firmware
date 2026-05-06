#include "m8n.h"
#include "../../fw/hardware_config.h"
#include "../../util/log.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include <math.h>
#include <string.h>

#define M8N_PROCESS_MAX_BYTES   512
#define M8N_UBX_MAX_PAYLOAD     512
#define M8N_UBX_SYNC_1          0xB5
#define M8N_UBX_SYNC_2          0x62
#define M8N_UBX_CLASS_ACK       0x05
#define M8N_UBX_ID_ACK_NACK     0x00
#define M8N_UBX_ID_ACK_ACK      0x01
#define M8N_UBX_CLASS_CFG       0x06
#define M8N_UBX_ID_CFG_CFG      0x09
#define M8N_UBX_ID_CFG_MSG      0x01
#define M8N_UBX_ID_CFG_GNSS     0x3E
#define M8N_UBX_ID_CFG_NAV5     0x24
#define M8N_UBX_ID_CFG_PRT      0x00
#define M8N_UBX_ID_CFG_RATE     0x08
#define M8N_UBX_ID_CFG_RST      0x04
#define M8N_UBX_CLASS_NAV       0x01
#define M8N_UBX_ID_NAV_PVT      0x07
#define M8N_UBX_CLASS_RXM       0x02
#define M8N_UBX_ID_RXM_PMREQ    0x41
#define M8N_UBX_CLASS_NMEA      0xF0
#define M8N_NAV_PVT_PAYLOAD_LEN 92
#define M8N_MIN_FIX_INTERVAL_MS 67
#define M8N_MAX_FIX_INTERVAL_MS 1000
#define M8N_GNSS_ID_GPS         0
#define M8N_GNSS_ID_SBAS        1
#define M8N_GNSS_ID_GALILEO     2
#define M8N_GNSS_ID_BEIDOU      3
#define M8N_GNSS_ID_QZSS        5
#define M8N_GNSS_ID_GLONASS     6

static volatile bool s_rx_enabled = false;

typedef enum {
    PS_IDLE,
    PS_UBX_SYNC2,
    PS_UBX_HEADER,
    PS_UBX_PAYLOAD,
    PS_UBX_CK_A,
    PS_UBX_CK_B,
} m8n_parse_state_t;

static m8n_parse_state_t s_parse_state = PS_IDLE;
static uint8_t s_ubx_header[4];
static size_t s_ubx_header_pos = 0;
static uint8_t s_ubx_payload[M8N_UBX_MAX_PAYLOAD];
static uint16_t s_ubx_payload_len = 0;
static uint16_t s_ubx_payload_pos = 0;
static uint8_t s_ubx_ck_a = 0;
static uint8_t s_ubx_ck_b = 0;
static uint8_t s_ubx_received_ck_a = 0;

static bool s_waiting_for_ack = false;
static uint8_t s_expected_ack_cls = 0;
static uint8_t s_expected_ack_id = 0;
static int8_t s_ack_result = -1;

static bool s_waiting_for_poll = false;
static uint8_t s_expected_poll_cls = 0;
static uint8_t s_expected_poll_id = 0;
static uint8_t *s_poll_buf = NULL;
static uint16_t s_poll_cap = 0;
static int s_poll_payload_len = -1;
static bool s_nav5_configured = false;

static void m8n_reset_parser(void) {
    s_parse_state = PS_IDLE;
    s_ubx_header_pos = 0;
    s_ubx_payload_len = 0;
    s_ubx_payload_pos = 0;
    s_ubx_ck_a = 0;
    s_ubx_ck_b = 0;
    s_ubx_received_ck_a = 0;
}

static void m8n_checksum_byte(uint8_t byte) {
    s_ubx_ck_a = (uint8_t)(s_ubx_ck_a + byte);
    s_ubx_ck_b = (uint8_t)(s_ubx_ck_b + s_ubx_ck_a);
}

static uint16_t m8n_read_u16(const uint8_t *payload, uint16_t offset) {
    return (uint16_t)payload[offset] | ((uint16_t)payload[offset + 1] << 8);
}

static uint32_t m8n_read_u32(const uint8_t *payload, uint16_t offset) {
    return (uint32_t)payload[offset] | ((uint32_t)payload[offset + 1] << 8) | ((uint32_t)payload[offset + 2] << 16) |
           ((uint32_t)payload[offset + 3] << 24);
}

static int32_t m8n_read_i32(const uint8_t *payload, uint16_t offset) { return (int32_t)m8n_read_u32(payload, offset); }

static void m8n_write_u16(uint8_t *payload, uint16_t offset, uint16_t value) {
    payload[offset] = (uint8_t)(value & 0xFF);
    payload[offset + 1] = (uint8_t)(value >> 8);
}

static void m8n_write_u32(uint8_t *payload, uint16_t offset, uint32_t value) {
    payload[offset] = (uint8_t)(value & 0xFF);
    payload[offset + 1] = (uint8_t)((value >> 8) & 0xFF);
    payload[offset + 2] = (uint8_t)((value >> 16) & 0xFF);
    payload[offset + 3] = (uint8_t)(value >> 24);
}

static void m8n_on_nav_pvt(const uint8_t *payload) {
    uint16_t year = m8n_read_u16(payload, 4);
    uint8_t month = payload[6];
    uint8_t day = payload[7];
    uint8_t hour = payload[8];
    uint8_t minute = payload[9];
    uint8_t second = payload[10];
    int32_t nano = m8n_read_i32(payload, 16);
    uint8_t fix_type = payload[20];
    uint8_t flags = payload[21];
    uint8_t num_sv = payload[23];
    int32_t lon_e7 = m8n_read_i32(payload, 24);
    int32_t lat_e7 = m8n_read_i32(payload, 28);
    int32_t height_mm = m8n_read_i32(payload, 32);
    uint32_t h_acc_mm = m8n_read_u32(payload, 40);
    uint32_t v_acc_mm = m8n_read_u32(payload, 44);
    int32_t ground_speed_mms = m8n_read_i32(payload, 60);
    int32_t heading_e5 = m8n_read_i32(payload, 64);
    uint16_t pdop = m8n_read_u16(payload, 76);

    int32_t millis = nano / 1000000;
    if (millis < 0) {
        millis = 0;
    }

    enum gps_fix_mode fix_mode = GPS_FIX_NONE;
    if (fix_type == 2) {
        fix_mode = GPS_FIX_2D;
    } else if (fix_type == 3 || fix_type == 4) {
        fix_mode = GPS_FIX_3D;
    }

    float h_acc_m = (float)h_acc_mm / 1000.0f;
    float v_acc_m = (float)v_acc_mm / 1000.0f;
    struct gps_telemetry built = {
        .date = (uint32_t)year * 10000 + (uint32_t)month * 100 + day,
        .time_ms = ((uint32_t)hour * 3600 + (uint32_t)minute * 60 + second) * 1000 + (uint32_t)millis,
        .latitude = (double)lat_e7 / 10000000.0,
        .longitude = (double)lon_e7 / 10000000.0,
        .altitude = (float)height_mm / 1000.0f,
        .speed = (float)ground_speed_mms / 1000.0f,
        .heading = (float)heading_e5 / 100000.0f,
        .fix_mode = fix_mode,
        .quality = (fix_type >= 2 && (flags & 0x01)) ? GPS_QUALITY_GPS_FIX : GPS_QUALITY_INVALID,
        .satellites = num_sv,
        .hdop = (float)pdop / 100.0f,
        .pdop = (float)pdop / 100.0f,
        .epe_2d = h_acc_m,
        .epe_3d = sqrtf(h_acc_m * h_acc_m + v_acc_m * v_acc_m),
        .last_update_ms = time_us_32() / 1000,
    };

    gps_update_fix_tracker(&gps, &built);
    if (gps.on_fix) {
        gps.on_fix(&built);
    }
}

static void m8n_on_ubx_frame(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len) {
    if (cls == M8N_UBX_CLASS_ACK && (id == M8N_UBX_ID_ACK_ACK || id == M8N_UBX_ID_ACK_NACK) && len == 2) {
        if (s_waiting_for_ack && payload[0] == s_expected_ack_cls && payload[1] == s_expected_ack_id) {
            s_ack_result = (id == M8N_UBX_ID_ACK_ACK) ? 1 : 0;
            s_waiting_for_ack = false;
        }
        return;
    }

    if (s_waiting_for_poll && cls == s_expected_poll_cls && id == s_expected_poll_id) {
        uint16_t copy_len = len < s_poll_cap ? len : s_poll_cap;
        if (copy_len > 0 && s_poll_buf != NULL) {
            memcpy(s_poll_buf, payload, copy_len);
        }
        s_poll_payload_len = len;
        s_waiting_for_poll = false;
    }

    if (cls == M8N_UBX_CLASS_NAV && id == M8N_UBX_ID_NAV_PVT && len >= M8N_NAV_PVT_PAYLOAD_LEN) {
        m8n_on_nav_pvt(payload);
    }
}

static void m8n_parse_byte(uint8_t byte) {
    switch (s_parse_state) {
        case PS_IDLE:
            if (byte == M8N_UBX_SYNC_1) {
                s_parse_state = PS_UBX_SYNC2;
            }
            break;

        case PS_UBX_SYNC2:
            if (byte == M8N_UBX_SYNC_2) {
                s_ubx_header_pos = 0;
                s_ubx_payload_len = 0;
                s_ubx_payload_pos = 0;
                s_ubx_ck_a = 0;
                s_ubx_ck_b = 0;
                s_parse_state = PS_UBX_HEADER;
            } else {
                s_parse_state = PS_IDLE;
            }
            break;

        case PS_UBX_HEADER:
            s_ubx_header[s_ubx_header_pos++] = byte;
            m8n_checksum_byte(byte);
            if (s_ubx_header_pos == sizeof(s_ubx_header)) {
                s_ubx_payload_len = (uint16_t)s_ubx_header[2] | ((uint16_t)s_ubx_header[3] << 8);
                if (s_ubx_payload_len > sizeof(s_ubx_payload)) {
                    m8n_reset_parser();
                } else if (s_ubx_payload_len == 0) {
                    s_parse_state = PS_UBX_CK_A;
                } else {
                    s_parse_state = PS_UBX_PAYLOAD;
                }
            }
            break;

        case PS_UBX_PAYLOAD:
            s_ubx_payload[s_ubx_payload_pos++] = byte;
            m8n_checksum_byte(byte);
            if (s_ubx_payload_pos == s_ubx_payload_len) {
                s_parse_state = PS_UBX_CK_A;
            }
            break;

        case PS_UBX_CK_A:
            s_ubx_received_ck_a = byte;
            s_parse_state = PS_UBX_CK_B;
            break;

        case PS_UBX_CK_B:
            if (s_ubx_received_ck_a == s_ubx_ck_a && byte == s_ubx_ck_b) {
                m8n_on_ubx_frame(s_ubx_header[0], s_ubx_header[1], s_ubx_payload, s_ubx_payload_len);
            }
            m8n_reset_parser();
            break;
    }
}

static void gps_uart_irq_handler(void) {
    while (uart_is_readable(gps.comm.uart.instance)) {
        uint8_t byte = uart_getc(gps.comm.uart.instance);
        struct gps_rx_buffer *buf = &gps.comm.uart.rx_buffer;
        uint16_t next_head = (buf->head + 1) & (GPS_RX_BUFFER_SIZE - 1);
        if (next_head != buf->tail) {
            buf->data[buf->head] = byte;
            buf->head = next_head;
        }
    }
}

static void m8n_reset_rx_buffer(struct gps_sensor *gps) {
    gps->comm.uart.rx_buffer.head = 0;
    gps->comm.uart.rx_buffer.tail = 0;
}

static void m8n_uart_start(struct gps_sensor *gps, uint baudrate) {
    gps->comm.uart.baudrate = baudrate;
    uart_deinit(gps->comm.uart.instance);
    uart_init(gps->comm.uart.instance, baudrate);
    uart_set_fifo_enabled(gps->comm.uart.instance, true);
    m8n_reset_rx_buffer(gps);
}

static void m8n_ubx_checksum_byte(uint8_t byte, uint8_t *ck_a, uint8_t *ck_b) {
    *ck_a = (uint8_t)(*ck_a + byte);
    *ck_b = (uint8_t)(*ck_b + *ck_a);
}

static void m8n_ubx_send(struct gps_sensor *gps, uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len) {
    uint8_t header[6] = {M8N_UBX_SYNC_1, M8N_UBX_SYNC_2, cls, id, (uint8_t)(len & 0xFF), (uint8_t)(len >> 8)};
    uint8_t ck_a = 0;
    uint8_t ck_b = 0;

    for (size_t i = 2; i < sizeof(header); i++) { m8n_ubx_checksum_byte(header[i], &ck_a, &ck_b); }
    for (uint16_t i = 0; i < len; i++) { m8n_ubx_checksum_byte(payload[i], &ck_a, &ck_b); }

    uart_write_blocking(gps->comm.uart.instance, header, sizeof(header));
    if (len > 0) {
        uart_write_blocking(gps->comm.uart.instance, payload, len);
    }
    uint8_t checksum[2] = {ck_a, ck_b};
    uart_write_blocking(gps->comm.uart.instance, checksum, sizeof(checksum));
}

static void m8n_process_for_ms(struct gps_sensor *gps, uint32_t window_ms) {
    absolute_time_t deadline = make_timeout_time_ms(window_ms);
    while (!time_reached(deadline)) {
        m8n_process(gps);
        tight_loop_contents();
    }
}

static void m8n_clear_wait_state(void) {
    s_waiting_for_ack = false;
    s_expected_ack_cls = 0;
    s_expected_ack_id = 0;
    s_ack_result = -1;

    s_waiting_for_poll = false;
    s_expected_poll_cls = 0;
    s_expected_poll_id = 0;
    s_poll_buf = NULL;
    s_poll_cap = 0;
    s_poll_payload_len = -1;
}

static bool m8n_send_cmd_wait(struct gps_sensor *gps, uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len,
                              uint32_t timeout_ms) {
    m8n_process(gps);
    s_expected_ack_cls = cls;
    s_expected_ack_id = id;
    s_ack_result = -1;
    s_waiting_for_ack = true;

    m8n_ubx_send(gps, cls, id, payload, len);

    absolute_time_t deadline = make_timeout_time_ms(timeout_ms);
    while (s_waiting_for_ack && !time_reached(deadline)) {
        m8n_process(gps);
        tight_loop_contents();
    }

    bool acked = (s_ack_result == 1);
    if (!acked) {
        LOG("M8N", "UBX %02X-%02X %s\n", cls, id, s_ack_result == 0 ? "NACK" : "ACK timeout");
    }
    s_waiting_for_ack = false;
    return acked;
}

static int m8n_poll_response(struct gps_sensor *gps, uint8_t cls, uint8_t id, const uint8_t *req_payload,
                             uint16_t req_len, uint8_t *out_buf, uint16_t out_cap, uint32_t timeout_ms) {
    m8n_process(gps);
    s_expected_poll_cls = cls;
    s_expected_poll_id = id;
    s_poll_buf = out_buf;
    s_poll_cap = out_cap;
    s_poll_payload_len = -1;
    s_waiting_for_poll = true;

    m8n_ubx_send(gps, cls, id, req_payload, req_len);

    absolute_time_t deadline = make_timeout_time_ms(timeout_ms);
    while (s_waiting_for_poll && !time_reached(deadline)) {
        m8n_process(gps);
        tight_loop_contents();
    }

    s_waiting_for_poll = false;
    if (s_poll_payload_len < 0) {
        LOG("M8N", "UBX %02X-%02X poll timeout\n", cls, id);
        return 0;
    }

    int payload_len = s_poll_payload_len;
    s_poll_buf = NULL;
    s_poll_cap = 0;

    m8n_process_for_ms(gps, 10);
    return payload_len;
}

static bool m8n_probe_link(struct gps_sensor *gps) {
    uint8_t payload[6];
    int response_len =
        m8n_poll_response(gps, M8N_UBX_CLASS_CFG, M8N_UBX_ID_CFG_RATE, NULL, 0, payload, sizeof(payload), 1000);
    if (response_len != (int)sizeof(payload)) {
        LOG("M8N", "UBX probe failed at configured baud=%u\n", gps->comm.uart.baudrate);
        return false;
    }

    LOG("M8N", "UBX probe OK at configured baud=%u\n", gps->comm.uart.baudrate);
    return true;
}

static uint16_t m8n_clamp_fix_interval(uint16_t fix_interval_ms) {
    if (fix_interval_ms < M8N_MIN_FIX_INTERVAL_MS) {
        return M8N_MIN_FIX_INTERVAL_MS;
    }
    if (fix_interval_ms > M8N_MAX_FIX_INTERVAL_MS) {
        return M8N_MAX_FIX_INTERVAL_MS;
    }
    return fix_interval_ms;
}

static bool m8n_configure_rate(struct gps_sensor *gps, uint16_t fix_interval_ms) {
    uint16_t meas_rate_ms = m8n_clamp_fix_interval(fix_interval_ms);
    uint8_t payload[6] = {0};

    m8n_write_u16(payload, 0, meas_rate_ms);
    m8n_write_u16(payload, 2, 1);
    m8n_write_u16(payload, 4, 1);

    LOG("M8N", "Set CFG-RATE measRate=%ums\n", meas_rate_ms);
    return m8n_send_cmd_wait(gps, M8N_UBX_CLASS_CFG, M8N_UBX_ID_CFG_RATE, payload, sizeof(payload), 1000);
}

static bool m8n_configure_msg_rate(struct gps_sensor *gps, uint8_t cls, uint8_t id, uint8_t uart_rate) {
    uint8_t payload[8] = {cls, id, 0, uart_rate, 0, 0, 0, 0};
    return m8n_send_cmd_wait(gps, M8N_UBX_CLASS_CFG, M8N_UBX_ID_CFG_MSG, payload, sizeof(payload), 1000);
}

static bool m8n_configure_messages(struct gps_sensor *gps) {
    static const uint8_t nmea_ids[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05};

    LOG("M8N", "Disable standard NMEA messages\n");
    for (size_t i = 0; i < sizeof(nmea_ids) / sizeof(nmea_ids[0]); i++) {
        if (!m8n_configure_msg_rate(gps, M8N_UBX_CLASS_NMEA, nmea_ids[i], 0)) {
            LOG("M8N", "Disable NMEA F0-%02X failed\n", nmea_ids[i]);
            return false;
        }
    }

    LOG("M8N", "Enable NAV-PVT\n");
    if (!m8n_configure_msg_rate(gps, M8N_UBX_CLASS_NAV, M8N_UBX_ID_NAV_PVT, 1)) {
        LOG("M8N", "Enable NAV-PVT failed\n");
        return false;
    }

    return true;
}

static bool m8n_configure_nav5(struct gps_sensor *gps) {
    if (s_nav5_configured) {
        return true;
    }

    uint8_t payload[36] = {0};
    m8n_write_u16(payload, 0, 0x0001);
    payload[2] = 4;

    LOG("M8N", "Set CFG-NAV5 dynModel=automotive\n");
    bool ok = m8n_send_cmd_wait(gps, M8N_UBX_CLASS_CFG, M8N_UBX_ID_CFG_NAV5, payload, sizeof(payload), 1000);
    if (ok) {
        s_nav5_configured = true;
    } else {
        LOG("M8N", "CFG-NAV5 failed\n");
    }
    return ok;
}

static void m8n_set_gnss_block_enabled(uint8_t *block, bool enabled) {
    if (enabled) {
        block[4] |= 0x01;
    } else {
        block[4] &= (uint8_t)~0x01;
    }
}

static const char *m8n_gnss_name(uint8_t gnss_id) {
    switch (gnss_id) {
        case M8N_GNSS_ID_GPS:
            return "GPS";
        case M8N_GNSS_ID_SBAS:
            return "SBAS";
        case M8N_GNSS_ID_GALILEO:
            return "Galileo";
        case M8N_GNSS_ID_BEIDOU:
            return "BeiDou";
        case M8N_GNSS_ID_QZSS:
            return "QZSS";
        case M8N_GNSS_ID_GLONASS:
            return "GLONASS";
        default:
            return NULL;
    }
}

static void m8n_append_gnss_name(char *mix, size_t mix_size, size_t *mix_len, const char *name, bool *first) {
    size_t sep_len = *first ? 0 : 2;
    size_t name_len = strlen(name);
    if (*mix_len + sep_len + name_len >= mix_size) {
        return;
    }

    if (!*first) {
        memcpy(&mix[*mix_len], ", ", sep_len);
        *mix_len += sep_len;
    }
    memcpy(&mix[*mix_len], name, name_len);
    *mix_len += name_len;
    mix[*mix_len] = '\0';
    *first = false;
}

static bool m8n_log_actual_gnss_mix(struct gps_sensor *gps) {
    uint8_t payload[M8N_UBX_MAX_PAYLOAD];
    int response_len =
        m8n_poll_response(gps, M8N_UBX_CLASS_CFG, M8N_UBX_ID_CFG_GNSS, NULL, 0, payload, sizeof(payload), 1000);
    if (response_len < 4 || response_len > (int)sizeof(payload)) {
        return false;
    }

    uint8_t num_blocks = payload[3];
    if (response_len < 4 + (int)num_blocks * 8) {
        LOG("M8N", "Accepted GNSS mix unavailable: malformed CFG-GNSS len=%d blocks=%u\n", response_len, num_blocks);
        return false;
    }

    char mix[64] = {0};
    size_t mix_len = 0;
    bool first = true;
    for (uint8_t i = 0; i < num_blocks; i++) {
        const uint8_t *block = &payload[4 + (size_t)i * 8];
        if ((block[4] & 0x01) == 0) {
            continue;
        }

        const char *name = m8n_gnss_name(block[0]);
        if (name == NULL) {
            continue;
        }

        m8n_append_gnss_name(mix, sizeof(mix), &mix_len, name, &first);
    }

    if (first) {
        strcpy(mix, "none");
    }

    LOG("M8N", "Accepted GNSS mix: %s\n", mix);
    return true;
}

static bool m8n_configure_gnss(struct gps_sensor *gps, bool gps_en, bool glonass_en, bool galileo_en, bool bds_en,
                               bool qzss_en) {
    (void)galileo_en;
    (void)qzss_en;

    uint8_t payload[M8N_UBX_MAX_PAYLOAD];
    int response_len =
        m8n_poll_response(gps, M8N_UBX_CLASS_CFG, M8N_UBX_ID_CFG_GNSS, NULL, 0, payload, sizeof(payload), 1000);
    if (response_len < 4 || response_len > (int)sizeof(payload)) {
        LOG("M8N", "CFG-GNSS poll failed or returned invalid length=%d\n", response_len);
        return false;
    }

    uint8_t num_blocks = payload[3];
    if (response_len < 4 + (int)num_blocks * 8) {
        LOG("M8N", "CFG-GNSS malformed: len=%d blocks=%u\n", response_len, num_blocks);
        return false;
    }

    bool want_glonass = glonass_en;
    bool want_beidou = bds_en;
    if (want_glonass && want_beidou) {
        LOG("M8N", "GLONASS and BeiDou requested; preferring GLONASS\n");
        want_beidou = false;
    }

    for (uint8_t i = 0; i < num_blocks; i++) {
        uint8_t *block = &payload[4 + (size_t)i * 8];
        switch (block[0]) {
            case M8N_GNSS_ID_GPS:
                m8n_set_gnss_block_enabled(block, gps_en);
                break;
            case M8N_GNSS_ID_SBAS:
                m8n_set_gnss_block_enabled(block, true);
                break;
            case M8N_GNSS_ID_GALILEO:
                m8n_set_gnss_block_enabled(block, false);
                break;
            case M8N_GNSS_ID_BEIDOU:
                m8n_set_gnss_block_enabled(block, want_beidou);
                break;
            case M8N_GNSS_ID_QZSS:
                m8n_set_gnss_block_enabled(block, true);
                break;
            case M8N_GNSS_ID_GLONASS:
                m8n_set_gnss_block_enabled(block, want_glonass);
                break;
            default:
                break;
        }
    }

    LOG("M8N", "Set CFG-GNSS blocks=%u\n", num_blocks);
    bool ok = m8n_send_cmd_wait(gps, M8N_UBX_CLASS_CFG, M8N_UBX_ID_CFG_GNSS, payload, (uint16_t)response_len, 1000);
    if (!ok) {
        LOG("M8N", "CFG-GNSS failed\n");
    }
    return ok;
}

static bool m8n_save_config(struct gps_sensor *gps) {
    uint8_t payload[13] = {0};
    m8n_write_u32(payload, 4, 0x0000FFFF);
    payload[12] = 0x01;

    LOG("M8N", "Save config to BBR\n");
    bool ok = m8n_send_cmd_wait(gps, M8N_UBX_CLASS_CFG, M8N_UBX_ID_CFG_CFG, payload, sizeof(payload), 1000);
    if (!ok) {
        LOG("M8N", "CFG-CFG save failed; continuing without persisted config\n");
    }
    return ok;
}

void m8n_send_command(struct gps_sensor *gps, const uint8_t *data, size_t len) {
    if (data != NULL && len > 0) {
        uart_write_blocking(gps->comm.uart.instance, data, len);
    }
}

void m8n_rx_enable(struct gps_sensor *gps) {
    m8n_reset_rx_buffer(gps);
    uart_set_irq_enables(gps->comm.uart.instance, true, false);
    irq_set_enabled(GPS_IRQ, true);
    s_rx_enabled = true;
}

void m8n_rx_disable(struct gps_sensor *gps) {
    irq_set_enabled(GPS_IRQ, false);
    uart_set_irq_enables(gps->comm.uart.instance, false, false);
    s_rx_enabled = false;
}

bool m8n_rx_is_enabled(void) { return s_rx_enabled; }

void m8n_init(struct gps_sensor *gps) {
    LOG("M8N", "Init: TX=%d RX=%d baud=%d\n", gps->comm.uart.tx_gpio, gps->comm.uart.rx_gpio, gps->comm.uart.baudrate);

    gpio_set_function(gps->comm.uart.tx_gpio, GPIO_FUNC_UART);
    gpio_set_function(gps->comm.uart.rx_gpio, GPIO_FUNC_UART);
    irq_set_exclusive_handler(GPS_IRQ, gps_uart_irq_handler);
    m8n_clear_wait_state();
    m8n_uart_start(gps, gps->comm.uart.baudrate);
    m8n_reset_parser();
    m8n_rx_enable(gps);

    m8n_process_for_ms(gps, 500);
    gps->available = m8n_probe_link(gps);
    if (gps->available) {
        return;
    }

    m8n_rx_disable(gps);
    uart_deinit(gps->comm.uart.instance);
    LOG("M8N", "Init failed at configured baud\n");
}

bool m8n_configure(struct gps_sensor *gps, uint16_t fix_interval_ms, bool gps_en, bool glonass_en, bool galileo_en,
                   bool bds_en, bool qzss_en) {
    if (!gps->available) {
        LOG("M8N", "Configure skipped: GPS unavailable\n");
        return false;
    }

    m8n_clear_wait_state();
    LOG("M8N", "Configure: fix=%dms gps=%d glo=%d gal=%d bds=%d qzss=%d\n", fix_interval_ms, gps_en, glonass_en,
        galileo_en, bds_en, qzss_en);

    bool nav5_ok = m8n_configure_nav5(gps);
    bool gnss_ok = false;
    bool rate_ok = false;
    bool messages_ok = false;

    if (nav5_ok) {
        gnss_ok = m8n_configure_gnss(gps, gps_en, glonass_en, galileo_en, bds_en, qzss_en);
    }
    if (nav5_ok && gnss_ok) {
        rate_ok = m8n_configure_rate(gps, fix_interval_ms);
    }
    if (nav5_ok && gnss_ok && rate_ok) {
        messages_ok = m8n_configure_messages(gps);
    }

    if (!nav5_ok || !gnss_ok || !rate_ok || !messages_ok) {
        LOG("M8N", "Configuration failed\n");
        return false;
    }

    bool save_ok = m8n_save_config(gps);
    m8n_log_actual_gnss_mix(gps);

    LOG("M8N", "Configuration complete (save=%d)\n", save_ok);
    return true;
}

void m8n_process(struct gps_sensor *gps) {
    struct gps_rx_buffer *buf = &gps->comm.uart.rx_buffer;
    uint16_t processed = 0;

    while (buf->tail != buf->head && processed < M8N_PROCESS_MAX_BYTES) {
        uint8_t byte = buf->data[buf->tail];
        buf->tail = (buf->tail + 1) & (GPS_RX_BUFFER_SIZE - 1);

        m8n_parse_byte(byte);
        processed++;
    }
}

bool m8n_hot_start(struct gps_sensor *gps) {
    uint8_t payload[4] = {0};
    payload[2] = 0x09;
    m8n_ubx_send(gps, M8N_UBX_CLASS_CFG, M8N_UBX_ID_CFG_RST, payload, sizeof(payload));
    return true;
}

bool m8n_cold_start(struct gps_sensor *gps) {
    uint8_t payload[4] = {0};
    m8n_write_u16(payload, 0, 0xFFFF);
    payload[2] = 0x09;
    m8n_ubx_send(gps, M8N_UBX_CLASS_CFG, M8N_UBX_ID_CFG_RST, payload, sizeof(payload));
    return true;
}

bool m8n_power_on(struct gps_sensor *gps) {
    uint8_t wake = 0xFF;
    uart_write_blocking(gps->comm.uart.instance, &wake, 1);
    sleep_ms(50);
    return true;
}

bool m8n_power_off(struct gps_sensor *gps) {
    uint8_t payload[8] = {0};
    m8n_write_u32(payload, 4, 0x00000002);
    m8n_ubx_send(gps, M8N_UBX_CLASS_RXM, M8N_UBX_ID_RXM_PMREQ, payload, sizeof(payload));
    return true;
}
