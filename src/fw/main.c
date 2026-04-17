#include <stdint.h>
#include <stdio.h>
#include <time.h>

#ifndef USB_UART_DEBUG
#include "bsp/board.h"
#endif

#include "ff.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/rosc.h"
#include "hardware/timer.h"
#include "pico/cyw43_arch.h"
#include "pico/multicore.h"
#include "pico/platform.h"
#include "pico/runtime_init.h"
#include "pico/sleep.h"
#include "pico/time.h"
#include "pico/types.h"

// For scb_hw so we can enable deep sleep
#include "hardware/structs/scb.h"

#include "../net/tcpserver.h"
#include "../net/wifi.h"
#include "../ntp//ntp.h"
#include "../rtc//ds3231.h"
#include "../sensor/travel/travel_sensor.h"
#include "../ui/pushbutton.h"
#include "../util/config.h"
#include "../util/list.h"
#include "../util/log.h"
#include "calibration_flow.h"
#include "core1_worker.h"
#include "data_acquisition.h"
#include "data_storage.h"
#include "display.h"
#include "fw_init.h"
#include "fw_state.h"
#include "helpers.h"
#include "live_core0_session.h"
#include "management_shared.h"
#include "sensor_setup.h"
#include "sst.h"
#include "state_views.h"

#include "../net/management_protocol.h"

#include "hardware_config.h"

volatile enum state state;
volatile bool marker_pending = false;
#if HAS_GPS
// Left press in GPS_WAIT is phase-dependent: before fix it skips GPS, after fix it confirms recording with GPS.
volatile bool skip_gps_recording = false;
static volatile bool gps_fix_ready = false;
static volatile bool confirm_gps_recording = false;
volatile uint8_t gps_last_satellites = 0;
volatile float gps_last_epe = 0.0f;
#endif

static struct fw_power_state power_state;

static ssd1306_t disp;
static volatile bool tcp_session_stop_requested = false;

struct ds3231 rtc;

static struct calibration_ctx cal_ctx;

// ----------------------------------------------------------------------------
// State handlers

// Enter a new recording session by resetting per-session state, reapplying calibration, and branching into GPS_WAIT or
// RECORD.
static void on_rec_start() {
    LOG("REC", "Starting recording session\n");
#if HAS_GPS
    skip_gps_recording = false;
    gps_fix_ready = false;
    confirm_gps_recording = false;
    gps_last_satellites = 0;
    gps_last_epe = 0.0f;
#endif
    recording_reset_buffers();

    display_message(&disp, "INIT SENS");
    if (!calibration_apply_to_sensors(&cal_ctx)) {
        LOG("REC", "No sensors available\n");
        display_message(&disp, "NO SENS");
        sleep_ms(1000);
        state = IDLE;
        return;
    }

#if HAS_GPS
    if (gps.available) {
        LOG("REC", "GPS available, entering GPS wait state\n");
        if (!gps.fix_tracker.ready) {
            gps.power_on(&gps);
        }
        state = GPS_WAIT;
        recording_start_gps_timer(&disp);
        return;
    }
#endif

    state = RECORD;
    recording_start(&disp);
}

#if HAS_GPS
static void on_gps_wait() {
    static absolute_time_t display_timeout = {0};

    // User pressed left while waiting for fix - skip GPS and record without it
    if (skip_gps_recording) {
        LOG("REC", "GPS skipped, starting recording without GPS\n");
        recording_stop_gps_timer();
        gps.power_off(&gps);
        LOG("REC", "GPS powered off\n");
        state = RECORD;
        recording_start(&disp);
        return;
    }

    // Check if fix just became ready
    if (gps.fix_tracker.ready && !gps_fix_ready) {
        LOG("REC", "GPS fix ready, waiting for user confirmation\n");
        gps_fix_ready = true;
    }

    // User confirmed GPS fix - start recording with GPS
    if (gps_fix_ready && confirm_gps_recording) {
        LOG("REC", "GPS confirmed, starting recording with GPS\n");
        recording_stop_gps_timer();
        state = RECORD;
        recording_start(&disp);
        return;
    }

    // Update display
    if (absolute_time_diff_us(get_absolute_time(), display_timeout) < 0) {
        display_timeout = make_timeout_time_ms(200);
        display_gps_wait_view(&disp, gps_fix_ready, gps_last_satellites, gps_last_epe);
    }
}
#endif

static void on_rec_stop() {
    LOG("REC", "Stopping recording\n");
    state = IDLE;
    display_message(&disp, "IDLE");
    recording_stop();
}

static void on_idle() {
    // No MSC if there is no USB cable connected, so checking
    // tud is not necessary.
    bool battery = on_battery();
    if (!battery && msc_present()) {
        soft_reset();
    }

    static absolute_time_t timeout = {0};
    if (absolute_time_diff_us(get_absolute_time(), timeout) < 0) {
        timeout = make_timeout_time_ms(500);

        uint8_t voltage_percentage = ((read_voltage() - BATTERY_MIN_V) / BATTERY_RANGE) * 100;
        static struct tm tz_tm;
        time_t t = rtc_timestamp();
        localtime_r(&t, &tz_tm);

        struct idle_view_model view_model = {
            .battery_power = battery,
            .voltage_percentage = voltage_percentage,
            .hour = (uint8_t)tz_tm.tm_hour,
            .minute = (uint8_t)tz_tm.tm_min,
            .fork_available = fork_sensor.check_availability(&fork_sensor),
            .shock_available = shock_sensor.check_availability(&shock_sensor),
#if HAS_IMU
            .imu_frame_available = imu_sensor_available(&imu_frame),
            .imu_fork_available = imu_sensor_available(&imu_fork),
#else
            .imu_frame_available = false,
            .imu_fork_available = false,
#endif
        };

        display_idle_view(&disp, &view_model);
    }
}

static void on_sleep() {
    LOG("POWER", "Entering sleep mode\n");
    log_close();
    sleep_run_from_xosc();
    display_message(&disp, "SLEEP.");

#if PICO_RP2040
    clocks_hw->sleep_en0 = CLOCKS_SLEEP_EN0_CLK_RTC_RTC_BITS;
    clocks_hw->sleep_en1 = 0x0;
#endif
    display_message(&disp, "SLEEP..");

#if PICO_RP2040
    scb_hw->scr = power_state.scb_orig | M0PLUS_SCR_SLEEPDEEP_BITS;
#else
    scb_hw->scr = power_state.scb_orig | M33_SCR_SLEEPDEEP_BITS;
#endif
    display_message(&disp, "SLEEP...");

    disable_button(BUTTON_LEFT, false);
    disable_button(BUTTON_RIGHT, true);
    ssd1306_poweroff(&disp);
    state = WAKING;
    __wfi();
}

static void on_waking() {
    LOG("POWER", "Waking from sleep\n");
    rosc_write(&rosc_hw->ctrl, ROSC_CTRL_ENABLE_BITS);

    scb_hw->scr = power_state.scb_orig;
    clocks_hw->sleep_en0 = power_state.clock0_orig;
    clocks_hw->sleep_en1 = power_state.clock1_orig;
    runtime_init_clocks();

    ssd1306_poweron(&disp);
    enable_button(BUTTON_LEFT);
    enable_button(BUTTON_RIGHT);
    state = IDLE;
}

static void on_msc() {
    if (!msc_present()) {
        soft_reset();
    }
    tud_task();
}

// GPS_WAIT is unreachable when GPS support is compiled out, but its enum slot still exists in the handler table.
static void on_disabled_gps() { tight_loop_contents(); }

// RECORD work runs from acquisition timers after recording_start, so the main loop only idles here.
static void on_rec() { tight_loop_contents(); }

static void service_management_core_requests(void) {
    int32_t result_code = MGMT_RESULT_INVALID_REQUEST;

    if (management_shared_get_state() != MGMT_CORE_STATE_REQUEST_READY) {
        return;
    }

    switch (management_core_shared.command) {
        case MGMT_CORE_CMD_APPLY_CONFIG: {
            struct config snapshot = management_core_shared.request.apply_config.snapshot;
            config_apply_snapshot(&snapshot);
            result_code = MGMT_RESULT_OK;
            break;
        }
        case MGMT_CORE_CMD_SET_TIME:
            result_code = set_system_time_utc((time_t)management_core_shared.request.set_time.utc_seconds,
                                              management_core_shared.request.set_time.micros)
                              ? MGMT_RESULT_OK
                              : MGMT_RESULT_INTERNAL_ERROR;
            break;
        case MGMT_CORE_CMD_NONE:
        default:
            result_code = MGMT_RESULT_INVALID_REQUEST;
            break;
    }

    management_shared_publish_response(result_code);
}

static void run_tcp_session(enum state session_state, const char *ready_message, bool allow_live_preview) {
    struct tcpserver_options tcp_options = {
        .allow_live_preview = allow_live_preview,
        .enable_mdns = true,
    };
    enum core1_dispatch_event event_id;
    int32_t event_data = 0;

    tcp_session_stop_requested = false;
    management_shared_reset();
    display_message(&disp, "CONNECT");
    if (!wifi_start_from_config(true)) {
        display_message(&disp, "CONN ERR");
        sleep_ms(1000);
        wifi_stop();
        state = IDLE;
        return;
    }

    core1_configure_tcp_server(&tcp_options);

    if (!core1_request_mode(CORE1_MODE_TCP_SERVER)) {
        display_message(&disp, "SRV BUSY");
        sleep_ms(1000);
        wifi_stop();
        state = IDLE;
        return;
    }

    if (!core1_wait_next_event(&event_id, &event_data)) {
        display_message(&disp, "SRV IPC");
        sleep_ms(1000);
        wifi_stop();
        state = IDLE;
        return;
    }

    if (event_id != CORE1_DISPATCH_EVENT_TCP_SERVER_READY) {
        display_message(&disp, "SRV ERR");
        sleep_ms(1000);
        wifi_stop();
        state = IDLE;
        return;
    }

    display_message(&disp, ready_message);
    while (state == session_state) {
        service_management_core_requests();

        if (allow_live_preview) {
            live_stream_core0_service();
        }

        if (tcp_session_stop_requested) {
            if (allow_live_preview) {
                live_stream_core0_stop();
            }
            core1_request_stop();
            tcp_session_stop_requested = false;
        }

        if (core1_poll_event(&event_id, &event_data)) {
            if (event_id == CORE1_DISPATCH_EVENT_BACKEND_COMPLETE || event_id == CORE1_DISPATCH_EVENT_BACKEND_ERROR) {
                break;
            }
        }

        tight_loop_contents();
        sleep_ms(1);
    }

    if (allow_live_preview) {
        live_stream_core0_stop();
    }
    wifi_stop();
    state = IDLE;
}

static void on_serve_tcp() { run_tcp_session(SERVE_TCP, "SERVER ON", true); }

static void on_sync_data() { run_tcp_session(SYNC_DATA, "READY DL", false); }

static void (*state_handlers[STATES_COUNT])() = {
    on_idle,      /* IDLE */
    on_sleep,     /* SLEEP */
    on_waking,    /* WAKING */
    on_rec_start, /* REC_START */
#if HAS_GPS
    on_gps_wait, /* GPS_WAIT */
#else
    on_disabled_gps, /* GPS_WAIT */
#endif
    on_rec,       /* RECORD */
    on_rec_stop,  /* REC_STOP */
    on_sync_data, /* SYNC_DATA */
    on_serve_tcp, /* SERVE_TCP */
    on_msc,       /* MSC */
};

// ----------------------------------------------------------------------------
// Button handlers

static void on_left_press(void *user_data) {
    switch (state) {
        case IDLE:
            state = REC_START;
            break;
        case RECORD:
            state = REC_STOP;
            break;
#if HAS_GPS
        case GPS_WAIT:
            if (gps_fix_ready) {
                confirm_gps_recording = true;
            } else {
                skip_gps_recording = true;
            }
            break;
#endif
        default:
            break;
    }
}

static void on_left_longpress(void *user_data) {
    switch (state) {
        case IDLE:
            state = SYNC_DATA;
            break;
        default:
            break;
    }
}

static void on_right_press(void *user_data) {
    switch (state) {
        case IDLE:
            state = SLEEP;
            break;
        case RECORD:
            marker_pending = true;
            LOG("REC", "Marker set\n");
            break;
        case SERVE_TCP:
            tcp_session_stop_requested = true;
            break;
        case SYNC_DATA:
            tcp_session_stop_requested = true;
            break;
        default:
            break;
    }
}

static void on_right_longpress(void *user_data) {
    switch (state) {
        case IDLE:
            state = SERVE_TCP;
            break;
        default:
            break;
    }
}

// ----------------------------------------------------------------------------
// Entry point

int main() {
    const struct fw_button_handlers button_handlers = {
        .on_left_press = on_left_press,
        .on_left_longpress = on_left_longpress,
        .on_right_press = on_right_press,
        .on_right_longpress = on_right_longpress,
    };

    state = fw_init(&disp, &rtc, &cal_ctx, &power_state, &button_handlers);

    while (true) { state_handlers[state](); }

    return 0;
}
