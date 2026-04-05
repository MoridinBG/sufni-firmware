#include "fw_init.h"

#ifndef USB_UART_DEBUG
#include "bsp/board.h"
#endif

#include "data_acquisition.h"
#include "data_storage.h"
#include "display.h"
#include "helpers.h"
#include "sensor_setup.h"

#include "../ntp/ntp.h"
#include "../pio_i2c/pio_i2c.h"
#include "../sensor/travel/travel_sensor.h"
#include "../ui/pushbutton.h"
#include "../util/config.h"
#include "../util/log.h"

#include "hardware/adc.h"
#include "hardware/structs/clocks.h"
#include "hardware/structs/scb.h"
#include "pico/cyw43_arch.h"
#include "pico/multicore.h"
#include "pico/platform.h"
#include "pico/runtime_init.h"
#include "pico/sleep.h"
#include "pico/stdlib.h"
#include "tusb.h"

#include <time.h>

static void halt_with_message(ssd1306_t *disp, const char *message) {
    display_message(disp, message);
    while (true) { tight_loop_contents(); }
}

static void init_board_io(void) {
#ifndef USB_UART_DEBUG
    board_init();
    tusb_init();
#else
    stdio_usb_init();
    sleep_ms(3000);
#endif
}

static void init_travel_sensors(void) {
    adc_init();
    fork_sensor.init(&fork_sensor);
    shock_sensor.init(&shock_sensor);

#if !defined(NDEBUG) && GPS_MODULE == GPS_NONE
    stdio_uart_init();
#endif
}

#if HAS_GPS
static void init_gps_sensor(void) {
    gps.on_fix = recording_on_gps_fix;
    if (gps_sensor_init(&gps)) {
        LOG("INIT", "GPS initialized\n");
        if (!gps_sensor_configure(&gps, 1000 / GPS_SAMPLE_RATE, true, true, true, true, false)) {
            LOG("INIT", "GPS configuration failed\n");
        }
        sleep_ms(50);
        gps.power_off(&gps);
        LOG("INIT", "GPS powered off to save power\n");
    } else {
        LOG("INIT", "GPS not found or failed to initialize\n");
    }
}
#endif

static void init_pio_i2c_bus(void) {
    uint offset = pio_add_program(I2C_PIO, &i2c_program);
    i2c_program_init(I2C_PIO, I2C_SM, offset, PIO_PIN_SDA, PIO_PIN_SDA + 1);
}

#if HAS_IMU
static void init_imu_sensors(void) {
#if IMU_FRAME != IMU_NONE
    if (!imu_sensor_init(&imu_frame)) {
        LOG("INIT", "Frame IMU not found or failed to initialize\n");
    }
#endif
#if IMU_FORK != IMU_NONE
    if (!imu_sensor_init(&imu_fork)) {
        LOG("INIT", "Fork IMU not found or failed to initialize\n");
    }
#endif
#if IMU_REAR != IMU_NONE
    if (!imu_sensor_init(&imu_rear)) {
        LOG("INIT", "Rear IMU not found or failed to initialize\n");
    }
#endif
}
#endif

static void init_rtc_and_aon_timer(ssd1306_t *disp, struct ds3231 *rtc) {
    struct tm tm_now;

    LOG("DS3231", "Initializing RTC\n");
    ds3231_init(rtc, I2C_PIO, I2C_SM, pio_i2c_write_blocking, pio_i2c_read_blocking);
    sleep_ms(1);

    LOG("DS3231", "Reading datetime\n");
    if (!ds3231_get_datetime(&rtc, &tm_now)) {
        LOG("DS3231", "RTC not connected\n");
        setup_display(&disp);
        halt_with_message(disp, "RTC ERR");
    }
    LOG("DS3231", "Time: %04d-%02d-%02d %02d:%02d:%02d\n", tm_now.tm_year + 1900, tm_now.tm_mon + 1, tm_now.tm_mday,
        tm_now.tm_hour, tm_now.tm_min, tm_now.tm_sec);

#if PICO_RP2040
    if (!aon_timer_start_calendar(&tm_now)) {
        setup_display(disp);
        halt_with_message(disp, "AON ERR");
    }
#else
    setenv("TZ", "UTC0", 1);
    tzset();
    time_t epoch = mktime(&tm_now);
    struct timespec ts = {.tv_sec = epoch, .tv_nsec = 0};
    if (!aon_timer_start(&ts)) {
        setup_display(disp);
        halt_with_message(disp, "AON ERR");
    }
#endif
}

static void init_storage(ssd1306_t *disp) {
    int err = setup_storage();
    if (err < 0) {
        setup_display(disp);
        halt_with_message(disp, "CARD ERR");
    }
    LOG("INIT", "Storage initialized\n");
}

static void init_runtime(ssd1306_t *disp) {
    if (!load_config()) {
        halt_with_message(disp, "CONF ERR");
    }
    LOG("INIT", "Config loaded\n");

    setup_ntp(config.ntp_server);
    cyw43_arch_init_with_country(config.country);
    setenv("TZ", config.timezone, 1);
    tzset();
    LOG("INIT", "WiFi initialized, country=%d, timezone=%s\n", config.country, config.timezone);
}

static void init_calibration_context(struct calibration_ctx *cal_ctx, ssd1306_t *disp) {
    *cal_ctx = (struct calibration_ctx){
        .fork = &fork_sensor,
        .shock = &shock_sensor,
#if HAS_IMU
        .imu_frame = &imu_frame,
        .imu_fork = &imu_fork,
        .imu_rear = &imu_rear,
#endif
        .disp = disp,
    };
}

static void run_calibration(struct calibration_ctx *cal_ctx) {
    if (calibration_check_needed(cal_ctx) && !calibration_run(cal_ctx)) {
        while (true) { tight_loop_contents(); }
    }

    calibration_apply_to_sensors(cal_ctx);
}

static void register_buttons(const struct fw_button_handlers *button_handlers) {
    create_button(BUTTON_LEFT, NULL, button_handlers->on_left_press, button_handlers->on_left_longpress);
    create_button(BUTTON_RIGHT, NULL, button_handlers->on_right_press, button_handlers->on_right_longpress);
}

enum state fw_init(ssd1306_t *disp, struct ds3231 *rtc, struct calibration_ctx *cal_ctx,
                   struct fw_power_state *power_state, const struct fw_button_handlers *button_handlers) {
    init_board_io();
    init_pio_i2c_bus();
    init_rtc_and_aon_timer(disp, rtc);
    init_storage(disp);
    log_init();

    init_travel_sensors();
#if HAS_GPS
    init_gps_sensor();
#endif
#if HAS_IMU
    init_imu_sensors();
#endif
    setup_display(disp);

#ifndef USB_UART_DEBUG
    if (msc_present()) {
        LOG("INIT", "Entering MSC mode\n");
        log_close();
        display_message(disp, "MSC MODE");
        return MSC;
    }
#endif

    multicore_launch_core1(&sd_writer_main);
    init_runtime(disp);

    // Sleep/wake restores these registers after deep sleep, so capture the post-init baseline once here.
    power_state->scb_orig = scb_hw->scr;
    power_state->clock0_orig = clocks_hw->sleep_en0;
    power_state->clock1_orig = clocks_hw->sleep_en1;

    init_calibration_context(cal_ctx, disp);
    run_calibration(cal_ctx);
    register_buttons(button_handlers);

    return IDLE;
}