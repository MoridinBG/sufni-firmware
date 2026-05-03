#include "data_acquisition.h"
#include "data_storage.h"

#include "core1_ipc.h"
#include "core1_worker.h"
#include "display.h"
#include "fw_state.h"
#include "sensor_setup.h"

#include "../ntp/ntp.h"
#include "../sensor/travel/travel_sensor.h"
#include "../util/config.h"
#include "../util/log.h"

#include "hardware/timer.h"
#include "pico/multicore.h"
#include "pico/platform.h"
#include "pico/time.h"

#include <stdint.h>
#include <stdio.h>

#if HAS_GPS
// Drain the UART often enough that the RX buffer stays below full without busy-polling.
static const int64_t GPS_RX_DRAIN_INTERVAL_US = -(int64_t)GPS_RX_BUFFER_SIZE * 1000000 / (GPS_BAUD_RATE / 10) / 2;
#endif

static struct travel_record travel_databuffer1[BUFFER_SIZE];
struct travel_record travel_databuffer2[BUFFER_SIZE];
static struct travel_record *active_travel_buffer = travel_databuffer1;
static uint16_t travel_count = 0;
static repeating_timer_t travel_timer;
static ssd1306_t *recording_disp = NULL;
static volatile bool recording_backend_failed = false;

#if HAS_GPS
static struct gps_record gps_databuffer1[GPS_BUFFER_SIZE];
struct gps_record gps_databuffer2[GPS_BUFFER_SIZE];
static struct gps_record *gps_active_buffer = gps_databuffer1;
static uint16_t gps_count = 0;
static repeating_timer_t gps_timer;
#endif

#if HAS_IMU
static const int64_t TEMPERATURE_SAMPLE_INTERVAL_US = -30000000LL;

static struct imu_record imu_databuffer1[IMU_BUFFER_SIZE];
struct imu_record imu_databuffer2[IMU_BUFFER_SIZE];
static struct imu_record *active_imu_buffer = imu_databuffer1;
static uint16_t imu_count = 0;
static repeating_timer_t imu_timer;
static struct temperature_record temperature_databuffer[TEMPERATURE_LOCATION_COUNT];
static repeating_timer_t temperature_timer;
static volatile bool temperature_timer_running = false;
#endif

static void recording_error(ssd1306_t *disp, const char *message) {
    display_message(disp, message);
    while (true) { tight_loop_contents(); }
}

static void recording_handle_storage_failure(int32_t backend_status) {
    if (recording_backend_failed) {
        return;
    }

    recording_backend_failed = true;
    if (storage_backend_status_is_fresult(backend_status)) {
        LOG("REC", "Storage backend failed: FatFS %ld\n", (long)storage_backend_status_to_fresult(backend_status));
    } else {
        LOG("REC", "Storage backend failed: %ld\n", (long)backend_status);
    }

#if HAS_GPS
    if (gps.available && !skip_gps_recording) {
        gps.power_off(&gps);
        LOG("REC", "GPS powered off\n");
    }
#endif

    state = IDLE;
    if (recording_disp != NULL) {
        display_message(recording_disp, "FILE ERR");
    }
}

static void storage_push_command(enum storage_session_command command) {
    multicore_fifo_push_blocking(CORE1_FIFO_WORD(CORE1_FIFO_FAMILY_STORAGE_CMD, command));
}

static bool storage_expect_event(enum storage_session_event expected_event) {
    uint32_t event_word = multicore_fifo_pop_blocking();

    if (core1_fifo_is_family(event_word, CORE1_FIFO_FAMILY_STORAGE_EVENT)) {
        if (CORE1_FIFO_ID(event_word) == (uint32_t)expected_event) {
            return true;
        }

        recording_handle_storage_failure(PICO_ERROR_GENERIC);
        return false;
    }

    if (core1_fifo_is_family(event_word, CORE1_FIFO_FAMILY_DISPATCH_EVENT)) {
        int32_t backend_status = (int32_t)multicore_fifo_pop_blocking();
        recording_handle_storage_failure(backend_status);
        return false;
    }

    recording_handle_storage_failure(PICO_ERROR_GENERIC);
    return false;
}

// Hand the filled buffer to core 1 and receive the now-free alternate buffer back for subsequent samples.
static bool dump_active_travel_buffer(uint16_t size) {
    if (recording_backend_failed) {
        return false;
    }

    storage_push_command(STORAGE_CMD_DUMP_TRAVEL);
    multicore_fifo_push_blocking(size);
    multicore_fifo_push_blocking((uintptr_t)active_travel_buffer);
    if (!storage_expect_event(STORAGE_EVENT_BUFFER_RETURNED)) {
        return false;
    }
    active_travel_buffer = (struct travel_record *)((uintptr_t)multicore_fifo_pop_blocking());
    return true;
}

#if HAS_GPS
static bool dump_gps_active_buffer(uint16_t size) {
    if (recording_backend_failed) {
        return false;
    }

    storage_push_command(STORAGE_CMD_DUMP_GPS);
    multicore_fifo_push_blocking(size);
    multicore_fifo_push_blocking((uintptr_t)gps_active_buffer);
    if (!storage_expect_event(STORAGE_EVENT_BUFFER_RETURNED)) {
        return false;
    }
    gps_active_buffer = (struct gps_record *)((uintptr_t)multicore_fifo_pop_blocking());
    return true;
}

void recording_on_gps_fix(const struct gps_telemetry *telemetry) {
    gps_last_satellites = telemetry->satellites;
    gps_last_epe = telemetry->epe_3d;

    if (gps.fix_tracker.ready) {
        LOG("GPS", "%.6f,%.6f alt=%.1f spd=%.1f sats=%d epe=%.1f\n", telemetry->latitude, telemetry->longitude,
            telemetry->altitude, telemetry->speed, telemetry->satellites, telemetry->epe_3d);

        if (state == RECORD) {
            if (gps_count == GPS_BUFFER_SIZE) {
                if (!dump_gps_active_buffer(GPS_BUFFER_SIZE)) {
                    return;
                }
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
static bool dump_active_imu_buffer(uint16_t size) {
    if (recording_backend_failed) {
        return false;
    }

    storage_push_command(STORAGE_CMD_DUMP_IMU);
    multicore_fifo_push_blocking(size);
    multicore_fifo_push_blocking((uintptr_t)active_imu_buffer);
    if (!storage_expect_event(STORAGE_EVENT_BUFFER_RETURNED)) {
        return false;
    }
    active_imu_buffer = (struct imu_record *)((uintptr_t)multicore_fifo_pop_blocking());
    return true;
}

static bool imu_temperature_supported(const struct imu_sensor *imu) {
    return imu->temperature_celsius != NULL || (imu->read_temperature != NULL && imu->temp_scale > 0.0f);
}

static bool imu_has_temperature(const struct imu_sensor *imu) {
    return imu->available && imu_temperature_supported(imu);
}

static uint16_t append_temperature_record(struct temperature_record *records, uint16_t count, uint8_t location_id,
                                          struct imu_sensor *imu, int64_t timestamp_utc) {
    if (!imu_has_temperature(imu)) {
        return count;
    }

    records[count].timestamp_utc = timestamp_utc;
    records[count].location_id = location_id;
    records[count].temperature_celsius = imu_sensor_get_temperature_celsius(imu);
    return count + 1;
}

static uint16_t collect_temperature_records(void) {
    uint16_t count = 0;
    int64_t timestamp_utc = (int64_t)rtc_timestamp();

    count =
        append_temperature_record(temperature_databuffer, count, TEMPERATURE_LOCATION_FRAME, &imu_frame, timestamp_utc);
    count =
        append_temperature_record(temperature_databuffer, count, TEMPERATURE_LOCATION_FORK, &imu_fork, timestamp_utc);
    count =
        append_temperature_record(temperature_databuffer, count, TEMPERATURE_LOCATION_REAR, &imu_rear, timestamp_utc);

    return count;
}

static bool recording_has_temperature_source(void) {
    return imu_has_temperature(&imu_frame) || imu_has_temperature(&imu_fork) || imu_has_temperature(&imu_rear);
}

static bool dump_temperature_records(uint16_t size) {
    if (recording_backend_failed) {
        return false;
    }
    if (size == 0) {
        return true;
    }

    storage_push_command(STORAGE_CMD_DUMP_TEMPERATURE);
    multicore_fifo_push_blocking(size);
    multicore_fifo_push_blocking((uintptr_t)temperature_databuffer);
    if (!storage_expect_event(STORAGE_EVENT_BUFFER_RETURNED)) {
        return false;
    }
    // Temperature uses a single scratch buffer, so there is no alternate buffer to swap in.
    (void)multicore_fifo_pop_blocking();
    return true;
}

static bool recording_capture_temperature(void) { return dump_temperature_records(collect_temperature_records()); }

static bool temperature_timer_cb(repeating_timer_t *rt) {
    (void)rt;

    if (state != RECORD) {
        temperature_timer_running = false;
        return false;
    }

    if (!recording_capture_temperature()) {
        temperature_timer_running = false;
        return false;
    }

    return true;
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
        if (!dump_active_imu_buffer(imu_count)) {
            return false;
        }
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
        if (!dump_active_travel_buffer(BUFFER_SIZE)) {
            return false;
        }
        travel_count = 0;
    }
    active_travel_buffer[travel_count].fork_angle = fork_sensor.measure(&fork_sensor);
    active_travel_buffer[travel_count].shock_angle = shock_sensor.measure(&shock_sensor);
    travel_count += 1;

    if (marker_pending) {
        // Flush in-flight Travel/IMU before emitting MARKER
        // so it ends up between complete data batches in the SST stream.
        if (!dump_active_travel_buffer(travel_count)) {
            return false;
        }
        travel_count = 0;
#if HAS_IMU
        if (!dump_active_imu_buffer(imu_count)) {
            return false;
        }
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
    temperature_timer_running = false;
#endif
}

// OPEN reserves the next SST file on core 1 and returns the inactive buffers that this core can start filling.
void recording_start(ssd1306_t *disp) {
    int32_t backend_status = 0;
    char msg[16];

    recording_backend_failed = false;
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
    if (!storage_expect_event(STORAGE_EVENT_OPEN_RESULT)) {
        recording_disp = NULL;
        return;
    }
    int index = (int)multicore_fifo_pop_blocking();
    if (index < 0) {
        if (storage_backend_status_is_fresult(index)) {
            LOG("REC", "Failed to open data file: FatFS %ld\n", (long)storage_backend_status_to_fresult(index));
        } else {
            LOG("REC", "Failed to open data file: %d\n", index);
        }
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

#if HAS_IMU
    bool temperature_active = recording_has_temperature_source();
    if (temperature_active && !recording_capture_temperature()) {
        recording_disp = NULL;
        return;
    }
#endif

    if (!add_repeating_timer_us(-1000000 / config.travel_sample_rate, travel_cb, NULL, &travel_timer)) {
        recording_error(disp, "TEL TMR ERR");
    }

#if HAS_IMU
    bool imu_active = imu_frame.available || imu_fork.available || imu_rear.available;
    if (imu_active && !add_repeating_timer_us(-1000000 / config.imu_sample_rate, imu_cb, NULL, &imu_timer)) {
        recording_error(disp, "IMU TMR ERR");
    }
    if (temperature_active &&
        !add_repeating_timer_us(TEMPERATURE_SAMPLE_INTERVAL_US, temperature_timer_cb, NULL, &temperature_timer)) {
        recording_error(disp, "TEMP TMR ERR");
    }
    temperature_timer_running = temperature_active;
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

#if HAS_IMU
    if (temperature_timer_running) {
        cancel_repeating_timer(&temperature_timer);
        temperature_timer_running = false;
    }
#endif

    cancel_repeating_timer(&travel_timer);
    if (!recording_backend_failed && travel_count > 0 && !dump_active_travel_buffer(travel_count)) {
        recording_disp = NULL;
        return;
    }

#if HAS_GPS
    cancel_repeating_timer(&gps_timer);
    if (!recording_backend_failed && gps_count > 0 && !dump_gps_active_buffer(gps_count)) {
        recording_disp = NULL;
        return;
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
        if (!recording_backend_failed && imu_count > 0 && !dump_active_imu_buffer(imu_count)) {
            recording_disp = NULL;
            return;
        }
    }
#endif

    if (recording_backend_failed) {
        recording_disp = NULL;
        return;
    }

    // FINISH only closes the file, so all remaining sensor chunks must already have been flushed.
    storage_push_command(STORAGE_CMD_FINISH);

    if (!core1_wait_for_event(CORE1_DISPATCH_EVENT_BACKEND_COMPLETE, &backend_status) || backend_status < 0) {
        recording_error(recording_disp, "FILE ERR");
    }

    recording_disp = NULL;
}
