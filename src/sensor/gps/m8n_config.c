#include "m8n_config.h"
#include "../../util/log.h"
#include "pico/time.h"
#include <string.h>

#define M8N_MIN_FIX_INTERVAL_MS 67
#define M8N_MAX_FIX_INTERVAL_MS 1000
#define M8N_GNSS_ID_GPS         0
#define M8N_GNSS_ID_SBAS        1
#define M8N_GNSS_ID_GALILEO     2
#define M8N_GNSS_ID_BEIDOU      3
#define M8N_GNSS_ID_QZSS        5
#define M8N_GNSS_ID_GLONASS     6

static bool s_nav5_configured = false;

static void m8n_config_service_for_ms(const struct m8n_ubx_io *io, uint32_t window_ms) {
    absolute_time_t deadline = make_timeout_time_ms(window_ms);
    while (!time_reached(deadline)) {
        io->service(io->service_context);
        tight_loop_contents();
    }
}

static bool m8n_send_cmd_wait(const struct m8n_ubx_io *io, uint8_t cls, uint8_t id, const uint8_t *payload,
                              uint16_t len, uint32_t timeout_ms) {
    enum ubx_ack_result result =
        ubx_send_ack_wait(io->ubx, io->gps, cls, id, payload, len, io->service, io->service_context, timeout_ms);
    if (result != UBX_ACK_ACK) {
        LOG("M8N", "UBX %02X-%02X %s\n", cls, id, result == UBX_ACK_NACK ? "NACK" : "ACK timeout");
    }
    return result == UBX_ACK_ACK;
}

static int m8n_poll_response(const struct m8n_ubx_io *io, uint8_t cls, uint8_t id, const uint8_t *req_payload,
                             uint16_t req_len, uint8_t *out_buf, uint16_t out_cap, uint32_t timeout_ms) {
    int payload_len = ubx_poll_response(io->ubx, io->gps, cls, id, req_payload, req_len, out_buf, out_cap, io->service,
                                        io->service_context, timeout_ms);
    if (payload_len < 0) {
        LOG("M8N", "UBX %02X-%02X poll timeout\n", cls, id);
        return 0;
    }

    m8n_config_service_for_ms(io, 10);
    return payload_len;
}

bool m8n_config_probe_link(const struct m8n_ubx_io *io) {
    uint8_t payload[6];
    int response_len = m8n_poll_response(io, UBX_CLASS_CFG, UBX_ID_CFG_RATE, NULL, 0, payload, sizeof(payload), 1000);
    if (response_len != (int)sizeof(payload)) {
        LOG("M8N", "UBX probe failed at baud=%u\n", io->gps->comm.uart.baudrate);
        return false;
    }

    LOG("M8N", "UBX probe OK at baud=%u\n", io->gps->comm.uart.baudrate);
    return true;
}

bool m8n_config_set_uart_baud(const struct m8n_ubx_io *io, uint target_baud) {
    uint current_baud = io->gps->comm.uart.baudrate;
    uint8_t port_id = 1;
    uint8_t payload[20];

    int response_len =
        m8n_poll_response(io, UBX_CLASS_CFG, UBX_ID_CFG_PRT, &port_id, sizeof(port_id), payload, sizeof(payload), 1000);
    if (response_len != (int)sizeof(payload)) {
        LOG("M8N", "CFG-PRT poll failed at baud=%u len=%d\n", current_baud, response_len);
        return false;
    }

    ubx_write_u32(payload, 8, target_baud);
    payload[12] |= 0x01;
    payload[14] |= 0x01;

    LOG("M8N", "Switch UART baud %u -> %u\n", current_baud, target_baud);
    return m8n_send_cmd_wait(io, UBX_CLASS_CFG, UBX_ID_CFG_PRT, payload, sizeof(payload), 1000);
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

static bool m8n_configure_rate(const struct m8n_ubx_io *io, uint16_t fix_interval_ms) {
    uint16_t meas_rate_ms = m8n_clamp_fix_interval(fix_interval_ms);
    uint8_t payload[6] = {0};

    ubx_write_u16(payload, 0, meas_rate_ms);
    ubx_write_u16(payload, 2, 1);
    ubx_write_u16(payload, 4, 1);

    LOG("M8N", "Set CFG-RATE measRate=%ums\n", meas_rate_ms);
    return m8n_send_cmd_wait(io, UBX_CLASS_CFG, UBX_ID_CFG_RATE, payload, sizeof(payload), 1000);
}

static bool m8n_configure_msg_rate(const struct m8n_ubx_io *io, uint8_t cls, uint8_t id, uint8_t uart_rate) {
    uint8_t payload[8] = {cls, id, 0, uart_rate, 0, 0, 0, 0};
    return m8n_send_cmd_wait(io, UBX_CLASS_CFG, UBX_ID_CFG_MSG, payload, sizeof(payload), 1000);
}

static bool m8n_configure_messages(const struct m8n_ubx_io *io) {
    static const uint8_t nmea_ids[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05};

    LOG("M8N", "Disable standard NMEA messages\n");
    for (size_t i = 0; i < sizeof(nmea_ids) / sizeof(nmea_ids[0]); i++) {
        if (!m8n_configure_msg_rate(io, UBX_CLASS_NMEA, nmea_ids[i], 0)) {
            LOG("M8N", "Disable NMEA F0-%02X failed\n", nmea_ids[i]);
            return false;
        }
    }

    LOG("M8N", "Enable NAV-PVT\n");
    if (!m8n_configure_msg_rate(io, UBX_CLASS_NAV, UBX_ID_NAV_PVT, 1)) {
        LOG("M8N", "Enable NAV-PVT failed\n");
        return false;
    }

    return true;
}

static bool m8n_configure_nav5(const struct m8n_ubx_io *io) {
    if (s_nav5_configured) {
        return true;
    }

    uint8_t payload[36] = {0};
    ubx_write_u16(payload, 0, 0x0001);
    payload[2] = 4;

    LOG("M8N", "Set CFG-NAV5 dynModel=automotive\n");
    bool ok = m8n_send_cmd_wait(io, UBX_CLASS_CFG, UBX_ID_CFG_NAV5, payload, sizeof(payload), 1000);
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

static bool m8n_log_actual_gnss_mix(const struct m8n_ubx_io *io) {
    uint8_t payload[UBX_MAX_PAYLOAD];
    int response_len = m8n_poll_response(io, UBX_CLASS_CFG, UBX_ID_CFG_GNSS, NULL, 0, payload, sizeof(payload), 1000);
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

static bool m8n_configure_gnss(const struct m8n_ubx_io *io, bool gps_en, bool glonass_en, bool galileo_en, bool bds_en,
                               bool qzss_en) {
    (void)galileo_en;
    (void)qzss_en;

    uint8_t payload[UBX_MAX_PAYLOAD];
    int response_len = m8n_poll_response(io, UBX_CLASS_CFG, UBX_ID_CFG_GNSS, NULL, 0, payload, sizeof(payload), 1000);
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
    bool ok = m8n_send_cmd_wait(io, UBX_CLASS_CFG, UBX_ID_CFG_GNSS, payload, (uint16_t)response_len, 1000);
    if (!ok) {
        LOG("M8N", "CFG-GNSS failed\n");
    }
    return ok;
}

static bool m8n_save_config(const struct m8n_ubx_io *io) {
    uint8_t payload[13] = {0};
    ubx_write_u32(payload, 4, 0x0000FFFF);
    payload[12] = 0x01;

    LOG("M8N", "Save config to BBR\n");
    bool ok = m8n_send_cmd_wait(io, UBX_CLASS_CFG, UBX_ID_CFG_CFG, payload, sizeof(payload), 1000);
    if (!ok) {
        LOG("M8N", "CFG-CFG save failed; continuing without persisted config\n");
    }
    return ok;
}

bool m8n_configure_receiver(const struct m8n_ubx_io *io, uint16_t fix_interval_ms, bool gps_en, bool glonass_en,
                            bool galileo_en, bool bds_en, bool qzss_en) {
    ubx_transport_clear_wait_state(io->ubx);
    LOG("M8N", "Configure: fix=%dms gps=%d glo=%d gal=%d bds=%d qzss=%d\n", fix_interval_ms, gps_en, glonass_en,
        galileo_en, bds_en, qzss_en);

    bool nav5_ok = m8n_configure_nav5(io);
    bool gnss_ok = false;
    bool rate_ok = false;
    bool messages_ok = false;

    if (nav5_ok) {
        gnss_ok = m8n_configure_gnss(io, gps_en, glonass_en, galileo_en, bds_en, qzss_en);
    }
    if (nav5_ok && gnss_ok) {
        rate_ok = m8n_configure_rate(io, fix_interval_ms);
    }
    if (nav5_ok && gnss_ok && rate_ok) {
        messages_ok = m8n_configure_messages(io);
    }

    if (!nav5_ok || !gnss_ok || !rate_ok || !messages_ok) {
        LOG("M8N", "Configuration failed\n");
        return false;
    }

    bool save_ok = m8n_save_config(io);
    m8n_log_actual_gnss_mix(io);

    LOG("M8N", "Configuration complete (save=%d)\n", save_ok);
    return true;
}
