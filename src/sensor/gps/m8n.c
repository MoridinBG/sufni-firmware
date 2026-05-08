#include "m8n.h"
#include "../../fw/hardware_config.h"
#include "../../util/log.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "m8n_config.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "ubx.h"
#include <math.h>

#define M8N_PROCESS_MAX_BYTES   512
#define M8N_NAV_PVT_PAYLOAD_LEN 92
#define M8N_BAUD_SAMPLE_MS      1200
#define M8N_WAKE_SETTLE_MS      100

struct m8n_autobaud_state {
    uint32_t ubx_count;
    uint32_t nmea_count;
    bool nmea_active;
    bool nmea_saw_cr;
    uint8_t nmea_len;
};

static volatile bool s_rx_enabled = false;
static struct ubx_transport s_ubx;
static struct m8n_autobaud_state s_autobaud;

static void m8n_ubx_service(void *context);

static struct m8n_ubx_io m8n_make_ubx_io(struct gps_sensor *gps) {
    return (struct m8n_ubx_io){
        .gps = gps,
        .ubx = &s_ubx,
        .service = m8n_ubx_service,
        .service_context = gps,
    };
}

static void m8n_reset_autobaud_counters(void) { s_autobaud = (struct m8n_autobaud_state){0}; }

static void m8n_autobaud_process_nmea_byte(uint8_t byte) {
    if (byte == '$') {
        s_autobaud.nmea_active = true;
        s_autobaud.nmea_saw_cr = false;
        s_autobaud.nmea_len = 1;
    } else if (s_autobaud.nmea_active) {
        if (byte == '\r') {
            s_autobaud.nmea_saw_cr = true;
        } else if (byte == '\n') {
            if (s_autobaud.nmea_saw_cr && s_autobaud.nmea_len >= 6) {
                s_autobaud.nmea_count++;
            }
            s_autobaud.nmea_active = false;
        } else if (s_autobaud.nmea_len < 90) {
            s_autobaud.nmea_len++;
        } else {
            s_autobaud.nmea_active = false;
        }
    }
}

static void m8n_on_nav_pvt(const uint8_t *payload) {
    uint16_t year = ubx_read_u16(payload, 4);
    uint8_t month = payload[6];
    uint8_t day = payload[7];
    uint8_t hour = payload[8];
    uint8_t minute = payload[9];
    uint8_t second = payload[10];
    int32_t nano = ubx_read_i32(payload, 16);
    uint8_t fix_type = payload[20];
    uint8_t flags = payload[21];
    uint8_t num_sv = payload[23];
    int32_t lon_e7 = ubx_read_i32(payload, 24);
    int32_t lat_e7 = ubx_read_i32(payload, 28);
    int32_t height_mm = ubx_read_i32(payload, 32);
    uint32_t h_acc_mm = ubx_read_u32(payload, 40);
    uint32_t v_acc_mm = ubx_read_u32(payload, 44);
    int32_t ground_speed_mms = ubx_read_i32(payload, 60);
    int32_t heading_e5 = ubx_read_i32(payload, 64);
    uint16_t pdop = ubx_read_u16(payload, 76);

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

static void m8n_on_ubx_frame(void *context, uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len) {
    (void)context;
    s_autobaud.ubx_count++;

    if (cls == UBX_CLASS_NAV && id == UBX_ID_NAV_PVT && len >= M8N_NAV_PVT_PAYLOAD_LEN) {
        m8n_on_nav_pvt(payload);
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

static void m8n_send_wake_byte(struct gps_sensor *gps) {
    uint8_t wake = 0xFF;
    uart_write_blocking(gps->comm.uart.instance, &wake, 1);
    sleep_ms(M8N_WAKE_SETTLE_MS);
}

static void m8n_process_for_ms(struct gps_sensor *gps, uint32_t window_ms) {
    absolute_time_t deadline = make_timeout_time_ms(window_ms);
    while (!time_reached(deadline)) {
        m8n_process(gps);
        tight_loop_contents();
    }
}

static void m8n_ubx_service(void *context) { m8n_process((struct gps_sensor *)context); }

static void m8n_reset_link_state(void) {
    ubx_transport_clear_wait_state(&s_ubx);
    ubx_transport_reset_parser(&s_ubx);
    m8n_reset_autobaud_counters();
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

static bool m8n_probe_link(struct gps_sensor *gps) {
    struct m8n_ubx_io io = m8n_make_ubx_io(gps);
    return m8n_config_probe_link(&io);
}

static bool m8n_try_baud(struct gps_sensor *gps, uint baudrate, bool wake_first) {
    if (s_rx_enabled) {
        m8n_rx_disable(gps);
    }

    m8n_reset_link_state();
    m8n_uart_start(gps, baudrate);
    m8n_rx_enable(gps);

    if (wake_first) {
        m8n_send_wake_byte(gps);
    }

    m8n_process_for_ms(gps, M8N_BAUD_SAMPLE_MS);
    LOG("M8N", "Baud %u sample: nmea=%lu ubx=%lu\n", baudrate, (unsigned long)s_autobaud.nmea_count,
        (unsigned long)s_autobaud.ubx_count);

    if (!m8n_probe_link(gps)) {
        m8n_rx_disable(gps);
        return false;
    }

    return true;
}

static bool m8n_autodetect_baud(struct gps_sensor *gps, uint configured_baud, bool wake_first) {
    if (m8n_try_baud(gps, configured_baud, wake_first)) {
        return true;
    }

    if (configured_baud != 9600 && m8n_try_baud(gps, 9600, wake_first)) {
        return true;
    }

    return false;
}

static bool m8n_switch_uart_baud(struct gps_sensor *gps, uint target_baud) {
    uint current_baud = gps->comm.uart.baudrate;
    struct m8n_ubx_io io = m8n_make_ubx_io(gps);
    if (!m8n_config_set_uart_baud(&io, target_baud)) {
        return false;
    }

    sleep_ms(100);
    m8n_rx_disable(gps);
    m8n_uart_start(gps, target_baud);
    m8n_reset_link_state();
    m8n_rx_enable(gps);

    if (m8n_probe_link(gps)) {
        return true;
    }

    m8n_rx_disable(gps);
    m8n_uart_start(gps, current_baud);
    ubx_transport_reset_parser(&s_ubx);
    m8n_rx_enable(gps);
    m8n_probe_link(gps);
    return false;
}

static bool m8n_detect_and_normalize_baud(struct gps_sensor *gps, uint configured_baud, bool wake_first) {
    if (!m8n_autodetect_baud(gps, configured_baud, wake_first)) {
        return false;
    }

    if (gps->comm.uart.baudrate != configured_baud && !m8n_switch_uart_baud(gps, configured_baud)) {
        LOG("M8N", "Continuing at detected baud=%u\n", gps->comm.uart.baudrate);
    }

    return true;
}

void m8n_init(struct gps_sensor *gps) {
    LOG("M8N", "Init: TX=%d RX=%d baud=%d\n", gps->comm.uart.tx_gpio, gps->comm.uart.rx_gpio, gps->comm.uart.baudrate);

    uint configured_baud = gps->comm.uart.baudrate;
    ubx_transport_init(&s_ubx, m8n_on_ubx_frame, NULL);
    gpio_set_function(gps->comm.uart.tx_gpio, GPIO_FUNC_UART);
    gpio_set_function(gps->comm.uart.rx_gpio, GPIO_FUNC_UART);
    irq_set_exclusive_handler(GPS_IRQ, gps_uart_irq_handler);

    gps->available = m8n_detect_and_normalize_baud(gps, configured_baud, true);
    if (gps->available) {
        return;
    }

    m8n_rx_disable(gps);
    uart_deinit(gps->comm.uart.instance);
    LOG("M8N", "Init failed: no response at supported baud\n");
}

bool m8n_configure(struct gps_sensor *gps, uint16_t fix_interval_ms, bool gps_en, bool glonass_en, bool galileo_en,
                   bool bds_en, bool qzss_en) {
    if (!gps->available) {
        LOG("M8N", "Configure skipped: GPS unavailable\n");
        return false;
    }

    struct m8n_ubx_io io = m8n_make_ubx_io(gps);
    return m8n_configure_receiver(&io, fix_interval_ms, gps_en, glonass_en, galileo_en, bds_en, qzss_en);
}

void m8n_process(struct gps_sensor *gps) {
    struct gps_rx_buffer *buf = &gps->comm.uart.rx_buffer;
    uint16_t processed = 0;

    while (buf->tail != buf->head && processed < M8N_PROCESS_MAX_BYTES) {
        uint8_t byte = buf->data[buf->tail];
        buf->tail = (buf->tail + 1) & (GPS_RX_BUFFER_SIZE - 1);

        m8n_autobaud_process_nmea_byte(byte);
        ubx_transport_process_byte(&s_ubx, byte);
        processed++;
    }
}

bool m8n_hot_start(struct gps_sensor *gps) {
    uint8_t payload[4] = {0};
    payload[2] = 0x09;
    ubx_send(gps, UBX_CLASS_CFG, UBX_ID_CFG_RST, payload, sizeof(payload));
    return true;
}

bool m8n_cold_start(struct gps_sensor *gps) {
    uint8_t payload[4] = {0};
    ubx_write_u16(payload, 0, 0xFFFF);
    payload[2] = 0x09;
    ubx_send(gps, UBX_CLASS_CFG, UBX_ID_CFG_RST, payload, sizeof(payload));
    return true;
}

bool m8n_power_on(struct gps_sensor *gps) {
    LOG("M8N", "Power on\n");
    bool ok = m8n_detect_and_normalize_baud(gps, GPS_BAUD_RATE, true);
    gps->available = ok;
    return ok;
}

bool m8n_power_off(struct gps_sensor *gps) {
    LOG("M8N", "Power off PMREQ at baud=%u\n", gps->comm.uart.baudrate);
    uint8_t payload[8] = {0};
    ubx_write_u32(payload, 4, 0x00000002);
    ubx_send(gps, UBX_CLASS_RXM, UBX_ID_RXM_PMREQ, payload, sizeof(payload));
    sleep_ms(50);
    m8n_rx_disable(gps);
    return true;
}
