#include "data_acquisition.h"

#include "core1_ipc.h"
#include "core1_worker.h"
#include "display.h"
#include "fw_state.h"
#include "sensor_setup.h"

#include "../sensor/travel/travel_sensor.h"
#include "../util/log.h"

#include "hardware/timer.h"
#include "pico/multicore.h"
#include "pico/platform.h"
#include "pico/time.h"

#include <stdint.h>
#include <stdio.h>

const uint16_t TRAVEL_SAMPLE_RATE = 1000;

#if HAS_IMU
const uint16_t IMU_SAMPLE_RATE = 1000;
#endif

#if HAS_GPS
const uint16_t GPS_SAMPLE_RATE = 3;
// Drain the UART often enough that the RX buffer stays below full without busy-polling.
static const int64_t GPS_RX_DRAIN_INTERVAL_US = -(int64_t)GPS_RX_BUFFER_SIZE * 1000000 / (GPS_BAUD_RATE / 10) / 2;
#endif

static struct travel_record travel_databuffer1[BUFFER_SIZE];
struct travel_record travel_databuffer2[BUFFER_SIZE];
static struct travel_record *active_travel_buffer = travel_databuffer1;
static uint16_t travel_count = 0;
static repeating_timer_t travel_timer;
static ssd1306_t *recording_disp = NULL;

#if HAS_GPS
static struct gps_record gps_databuffer1[GPS_BUFFER_SIZE];
struct gps_record gps_databuffer2[GPS_BUFFER_SIZE];
static struct gps_record *gps_active_buffer = gps_databuffer1;
static uint16_t gps_count = 0;
static repeating_timer_t gps_timer;
#endif

#if HAS_IMU
static struct imu_record imu_databuffer1[IMU_BUFFER_SIZE];
struct imu_record imu_databuffer2[IMU_BUFFER_SIZE];
static struct imu_record *active_imu_buffer = imu_databuffer1;
static uint16_t imu_count = 0;
static repeating_timer_t imu_timer;
#endif

static void recording_error(ssd1306_t *disp, const char *message) {
    display_message(disp, message);
    while (true) { tight_loop_contents(); }
}

static void storage_push_command(enum storage_session_command command) {
    multicore_fifo_push_blocking(CORE1_FIFO_WORD(CORE1_FIFO_FAMILY_STORAGE_CMD, command));
}

static void storage_expect_event(enum storage_session_event expected_event) {
    uint32_t event_word = multicore_fifo_pop_blocking();
    if (!core1_fifo_is_family(event_word, CORE1_FIFO_FAMILY_STORAGE_EVENT) ||
        CORE1_FIFO_ID(event_word) != (uint32_t)expected_event) {
        recording_error(recording_disp, "STO IPC ERR");
    }
}

// Hand the filled buffer to core 1 and receive the now-free alternate buffer back for subsequent samples.
static void dump_active_travel_buffer(uint16_t size) {
    storage_push_command(STORAGE_CMD_DUMP_TRAVEL);
    multicore_fifo_push_blocking(size);
    multicore_fifo_push_blocking((uintptr_t)active_travel_buffer);
    storage_expect_event(STORAGE_EVENT_BUFFER_RETURNED);
    active_travel_buffer = (struct travel_record *)((uintptr_t)multicore_fifo_pop_blocking());
}

#if HAS_GPS
static void dump_gps_active_buffer(uint16_t size) {
    storage_push_command(STORAGE_CMD_DUMP_GPS);
    multicore_fifo_push_blocking(size);
    multicore_fifo_push_blocking((uintptr_t)gps_active_buffer);
    storage_expect_event(STORAGE_EVENT_BUFFER_RETURNED);
    gps_active_buffer = (struct gps_record *)((uintptr_t)multicore_fifo_pop_blocking());
}

void recording_on_gps_fix(const struct gps_telemetry *telemetry) {
    gps_last_satellites = telemetry->satellites;
    gps_last_epe = telemetry->epe_3d;

    if (gps.fix_tracker.ready) {
        LOG("GPS", "%.6f,%.6f alt=%.1f spd=%.1f sats=%d epe=%.1f\n", telemetry->latitude, telemetry->longitude,
            telemetry->altitude, telemetry->speed, telemetry->satellites, telemetry->epe_3d);

        if (state == RECORD) {
            if (gps_count == GPS_BUFFER_SIZE) {
                dump_gps_active_buffer(GPS_BUFFER_SIZE);
                gps_count = 0;
            }

            gps_active_buffer[gps_count].date = telemetry->date;
            gps_active_buffer[gps_count].time_ms = telemetry->time_ms;
            gps_active_buffer[gps_count].latitude = telemetry->latitude;
            gps_active_buffer[gps_count].longitude = telemetry->longitude;
            gps_active_buffer[gps_count].altitude = telemetry->altitude;
            gps_active_buffer[gps_count].speed = telemetry->speed;
            gps_active_buffer[gps_count].heading = telemetry->heading;
            gps_active_buffer[gps_count].fix_mode = (uint8_t)telemetry->fix_mode;
            gps_active_buffer[gps_count].satellites = telemetry->satellites;
            gps_active_buffer[gps_count].epe_2d = telemetry->epe_2d;
            gps_active_buffer[gps_count].epe_3d = telemetry->epe_3d;
            gps_count++;
        }
    } else {
        LOG("GPS", "No reliable fix. sats=%d epe=%.1f\n", telemetry->satellites, telemetry->epe_3d);
    }
}

static bool gps_timer_cb(repeating_timer_t *rt) {
    if (gps.available) {
        gps.process(&gps);
    }
    // Keep draining GPS bytes while waiting for a fix so GPS_WAIT sees fresh fix status and telemetry.
    return state == RECORD || state == GPS_WAIT;
}

void recording_start_gps_timer(ssd1306_t *disp) {
    if (!add_repeating_timer_us(GPS_RX_DRAIN_INTERVAL_US, gps_timer_cb, NULL, &gps_timer)) {
        recording_error(disp, "GPS TMR ERR");
    }
}

void recording_stop_gps_timer(void) { cancel_repeating_timer(&gps_timer); }
#endif

#if HAS_IMU
static void dump_active_imu_buffer(uint16_t size) {
    storage_push_command(STORAGE_CMD_DUMP_IMU);
    multicore_fifo_push_blocking(size);
    multicore_fifo_push_blocking((uintptr_t)active_imu_buffer);
    storage_expect_event(STORAGE_EVENT_BUFFER_RETURNED);
    active_imu_buffer = (struct imu_record *)((uintptr_t)multicore_fifo_pop_blocking());
}

static bool imu_cb(repeating_timer_t *rt) {
    uint8_t active_count = 0;
    if (imu_frame.available)
        active_count++;
    if (imu_fork.available)
        active_count++;
    if (imu_rear.available)
        active_count++;

    if (imu_count + active_count > IMU_BUFFER_SIZE) {
        dump_active_imu_buffer(imu_count);
        imu_count = 0;
    }

    int16_t ax, ay, az, gx, gy, gz;
    if (imu_frame.available) {
        imu_sensor_read(&imu_frame, &ax, &ay, &az, &gx, &gy, &gz);
        active_imu_buffer[imu_count].ax = ax;
        active_imu_buffer[imu_count].ay = ay;
        active_imu_buffer[imu_count].az = az;
        active_imu_buffer[imu_count].gx = gx;
        active_imu_buffer[imu_count].gy = gy;
        active_imu_buffer[imu_count].gz = gz;
        imu_count++;
    }
    if (imu_fork.available) {
        imu_sensor_read(&imu_fork, &ax, &ay, &az, &gx, &gy, &gz);
        active_imu_buffer[imu_count].ax = ax;
        active_imu_buffer[imu_count].ay = ay;
        active_imu_buffer[imu_count].az = az;
        active_imu_buffer[imu_count].gx = gx;
        active_imu_buffer[imu_count].gy = gy;
        active_imu_buffer[imu_count].gz = gz;
        imu_count++;
    }
    if (imu_rear.available) {
        imu_sensor_read(&imu_rear, &ax, &ay, &az, &gx, &gy, &gz);
        active_imu_buffer[imu_count].ax = ax;
        active_imu_buffer[imu_count].ay = ay;
        active_imu_buffer[imu_count].az = az;
        active_imu_buffer[imu_count].gx = gx;
        active_imu_buffer[imu_count].gy = gy;
        active_imu_buffer[imu_count].gz = gz;
        imu_count++;
    }

    return state == RECORD;
}
#endif

static bool travel_cb(repeating_timer_t *rt) {
    if (travel_count == BUFFER_SIZE) {
        dump_active_travel_buffer(BUFFER_SIZE);
        travel_count = 0;
    }
    active_travel_buffer[travel_count].fork_angle = fork_sensor.measure(&fork_sensor);
    active_travel_buffer[travel_count].shock_angle = shock_sensor.measure(&shock_sensor);
    travel_count += 1;

    if (marker_pending) {
        // Flush in-flight Travel/IMU before emitting MARKER
        // so it ends up between complete data batches in the SST stream.
        dump_active_travel_buffer(travel_count);
        travel_count = 0;
#if HAS_IMU
        dump_active_imu_buffer(imu_count);
        imu_count = 0;
#endif

        storage_push_command(STORAGE_CMD_MARKER);
        marker_pending = false;
    }

    return state == RECORD;
}

void recording_reset_buffers(void) {
    active_travel_buffer = travel_databuffer1;
    travel_count = 0;

#if HAS_GPS
    gps_active_buffer = gps_databuffer1;
    gps_count = 0;
#endif

#if HAS_IMU
    active_imu_buffer = imu_databuffer1;
    imu_count = 0;
#endif

}

// OPEN reserves the next SST file on core 1 and returns the inactive buffers that this core can start filling.
void recording_start(ssd1306_t *disp) {
    int32_t backend_status = 0;
    char msg[16];

    recording_disp = disp;
    sprintf(msg, "REC:%s|%s", fork_sensor.available ? "F" : ".", shock_sensor.available ? "S" : ".");
    display_message(disp, msg);

    if (!core1_request_mode(CORE1_MODE_STORAGE)) {
        recording_error(disp, "STO BUSY");
    }

    if (!core1_wait_for_event(CORE1_DISPATCH_EVENT_STORAGE_READY, &backend_status)) {
        recording_error(disp, "STO IPC ERR");
    }

    storage_push_command(STORAGE_CMD_OPEN);
    storage_expect_event(STORAGE_EVENT_OPEN_RESULT);
    int index = (int)multicore_fifo_pop_blocking();
    if (index < 0) {
        LOG("REC", "Failed to open data file\n");
        recording_error(disp, "FILE ERR");
    }

    active_travel_buffer = (struct travel_record *)((uintptr_t)multicore_fifo_pop_blocking());
#if HAS_IMU
    active_imu_buffer = (struct imu_record *)((uintptr_t)multicore_fifo_pop_blocking());
#endif
#if HAS_GPS
    gps_active_buffer = (struct gps_record *)((uintptr_t)multicore_fifo_pop_blocking());
#endif
    LOG("REC", "Recording to file index %d\n", index);

    if (!add_repeating_timer_us(-1000000 / TRAVEL_SAMPLE_RATE, travel_cb, NULL, &travel_timer)) {
        recording_error(disp, "TEL TMR ERR");
    }

#if HAS_IMU
    bool imu_active = imu_frame.available || imu_fork.available || imu_rear.available;
    if (imu_active && !add_repeating_timer_us(-1000000 / IMU_SAMPLE_RATE, imu_cb, NULL, &imu_timer)) {
        recording_error(disp, "IMU TMR ERR");
    }
#endif

#if HAS_GPS
    if (gps.available && !skip_gps_recording &&
        !add_repeating_timer_us(GPS_RX_DRAIN_INTERVAL_US, gps_timer_cb, NULL, &gps_timer)) {
        recording_error(disp, "GPS TMR ERR");
    }
#endif
}

void recording_stop(void) {
    int32_t backend_status = 0;

    cancel_repeating_timer(&travel_timer);
    if (travel_count > 0) {
        dump_active_travel_buffer(travel_count);
    }

#if HAS_GPS
    cancel_repeating_timer(&gps_timer);
    if (gps_count > 0) {
        dump_gps_active_buffer(gps_count);
    }
    if (gps.available && !skip_gps_recording) {
        gps.power_off(&gps);
        LOG("REC", "GPS powered off\n");
    }
#endif

#if HAS_IMU
    bool imu_active = imu_frame.available || imu_fork.available || imu_rear.available;
    if (imu_active) {
        cancel_repeating_timer(&imu_timer);
        if (imu_count > 0) {
            dump_active_imu_buffer(imu_count);
        }
    }
#endif

    // FINISH only closes the file, so all remaining sensor chunks must already have been flushed.
    storage_push_command(STORAGE_CMD_FINISH);

    if (!core1_wait_for_event(CORE1_DISPATCH_EVENT_BACKEND_COMPLETE, &backend_status) || backend_status < 0) {
        recording_error(recording_disp, "FILE ERR");
    }

    recording_disp = NULL;
}