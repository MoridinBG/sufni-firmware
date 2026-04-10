#include "live_stream_core0.h"

#include "data_acquisition.h"
#include "sensor_setup.h"

#include "../net/live_protocol.h"
#include "../ntp/ntp.h"
#include "../sensor/imu/imu_sensor.h"
#include "../sensor/travel/travel_sensor.h"

#include "hardware/timer.h"
#include "pico/time.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#if HAS_GPS
static const int64_t LIVE_GPS_RX_DRAIN_INTERVAL_US = -(int64_t)GPS_RX_BUFFER_SIZE * 1000000 / (GPS_BAUD_RATE / 10) / 2;
#endif

struct live_runtime_state {
    bool active;
    bool travel_enabled;
    bool imu_enabled;
    bool gps_enabled;
    uint32_t publish_cadence_us;
    uint32_t travel_period_us;
    uint32_t imu_period_us;
    uint32_t gps_fix_interval_ms;
    uint32_t active_imu_count;
    uint32_t active_imu_mask;
    int current_travel_slot;
    int current_imu_slot;
    int current_gps_slot;
    repeating_timer_t travel_timer;
    repeating_timer_t imu_timer;
#if HAS_GPS
    repeating_timer_t gps_timer;
#endif
};

static struct live_runtime_state live_runtime = {
    .current_travel_slot = -1,
    .current_imu_slot = -1,
    .current_gps_slot = -1,
};

static struct live_slot_header *live_slot_header_at(void *base, size_t stride, uint32_t slot_index) {
    return (struct live_slot_header *)((uint8_t *)base + (stride * slot_index));
}

static int live_claim_slot(void *base, size_t stride, uint32_t slot_count, volatile uint32_t *dropped_batches) {
    uint32_t slot_index;
    uint32_t oldest_sequence = UINT32_MAX;
    int oldest_ready_index = -1;

    for (slot_index = 0; slot_index < slot_count; ++slot_index) {
        struct live_slot_header *header = live_slot_header_at(base, stride, slot_index);
        uint32_t expected_state = LIVE_SLOT_FREE;
        if (__atomic_compare_exchange_n(&header->state, &expected_state, LIVE_SLOT_FILLING, false, __ATOMIC_ACQ_REL,
                                        __ATOMIC_ACQUIRE)) {
            return (int)slot_index;
        }
    }

    for (slot_index = 0; slot_index < slot_count; ++slot_index) {
        struct live_slot_header *header = live_slot_header_at(base, stride, slot_index);
        if (header->state == LIVE_SLOT_READY && header->sequence <= oldest_sequence) {
            oldest_sequence = header->sequence;
            oldest_ready_index = (int)slot_index;
        }
    }

    if (oldest_ready_index >= 0) {
        struct live_slot_header *header = live_slot_header_at(base, stride, (uint32_t)oldest_ready_index);
        uint32_t expected_state = LIVE_SLOT_READY;
        if (__atomic_compare_exchange_n(&header->state, &expected_state, LIVE_SLOT_FILLING, false, __ATOMIC_ACQ_REL,
                                        __ATOMIC_ACQUIRE)) {
            (*dropped_batches)++;
            return oldest_ready_index;
        }
    }

    return -1;
}

static void live_init_slot_header(struct live_slot_header *header, struct live_stream_stats *stats,
                                  uint64_t start_time_us) {
    memset(header, 0, sizeof(*header));
    header->state = LIVE_SLOT_FILLING;
    header->sequence = stats->next_sequence++;
    header->first_index = stats->next_index;
    header->first_monotonic_us = start_time_us;
}

static void live_publish_slot(struct live_slot_header *header, enum live_stream_type stream_type,
                              struct live_stream_stats *stats) {
    if (header == NULL || header->sample_count == 0) {
        if (header != NULL) {
            live_stream_release_slot(&header->state);
        }
        return;
    }

    header->queue_depth_after_publish = live_stream_shared_ready_depth(stream_type) + 1;
    header->dropped_batches = stats->dropped_batches;
    live_stream_publish_slot(&header->state);
    stats->queue_depth = live_stream_shared_ready_depth(stream_type);
}

static int live_begin_travel_slot(uint64_t now_us) {
    int slot_index = live_claim_slot(live_stream_shared.travel_slots, sizeof(struct live_travel_slot),
                                     LIVE_TRAVEL_SLOT_COUNT, &live_stream_shared.travel_stats.dropped_batches);
    if (slot_index >= 0) {
        live_init_slot_header(&live_stream_shared.travel_slots[slot_index].header, &live_stream_shared.travel_stats,
                              now_us);
    }
    return slot_index;
}

#if HAS_IMU
static int live_begin_imu_slot(uint64_t now_us) {
    int slot_index = live_claim_slot(live_stream_shared.imu_slots, sizeof(struct live_imu_slot), LIVE_IMU_SLOT_COUNT,
                                     &live_stream_shared.imu_stats.dropped_batches);
    if (slot_index >= 0) {
        live_init_slot_header(&live_stream_shared.imu_slots[slot_index].header, &live_stream_shared.imu_stats, now_us);
    }
    return slot_index;
}
#endif

#if HAS_GPS
static int live_begin_gps_slot(uint64_t now_us) {
    int slot_index = live_claim_slot(live_stream_shared.gps_slots, sizeof(struct live_gps_slot), LIVE_GPS_SLOT_COUNT,
                                     &live_stream_shared.gps_stats.dropped_batches);
    if (slot_index >= 0) {
        live_init_slot_header(&live_stream_shared.gps_slots[slot_index].header, &live_stream_shared.gps_stats, now_us);
    }
    return slot_index;
}
#endif

static bool live_travel_cb(repeating_timer_t *timer) {
    uint64_t now_us = time_us_64();
    struct live_travel_slot *slot;

    if (!live_runtime.active || !live_runtime.travel_enabled) {
        return false;
    }

    if (live_runtime.current_travel_slot < 0) {
        live_runtime.current_travel_slot = live_begin_travel_slot(now_us);
        if (live_runtime.current_travel_slot < 0) {
            return true;
        }
    }

    slot = &live_stream_shared.travel_slots[live_runtime.current_travel_slot];
    if ((slot->header.sample_count > 0 &&
         (now_us - slot->header.first_monotonic_us) >= live_runtime.publish_cadence_us) ||
        slot->header.sample_count >= LIVE_TRAVEL_SAMPLES_PER_SLOT) {
        live_publish_slot(&slot->header, LIVE_STREAM_TYPE_TRAVEL, &live_stream_shared.travel_stats);
        live_runtime.current_travel_slot = live_begin_travel_slot(now_us);
        if (live_runtime.current_travel_slot < 0) {
            return true;
        }
        slot = &live_stream_shared.travel_slots[live_runtime.current_travel_slot];
    }

    slot->payload[slot->header.sample_count].fork_angle = fork_sensor.measure(&fork_sensor);
    slot->payload[slot->header.sample_count].shock_angle = shock_sensor.measure(&shock_sensor);
    slot->header.sample_count++;
    slot->header.payload_bytes = slot->header.sample_count * sizeof(struct travel_record);
    live_stream_shared.travel_stats.next_index++;

    return true;
}

#if HAS_IMU
static bool live_imu_cb(repeating_timer_t *timer) {
    uint64_t now_us = time_us_64();
    struct live_imu_slot *slot;
    uint32_t record_index;
    int16_t ax, ay, az, gx, gy, gz;

    if (!live_runtime.active || !live_runtime.imu_enabled) {
        return false;
    }

    if (live_runtime.current_imu_slot < 0) {
        live_runtime.current_imu_slot = live_begin_imu_slot(now_us);
        if (live_runtime.current_imu_slot < 0) {
            return true;
        }
    }

    slot = &live_stream_shared.imu_slots[live_runtime.current_imu_slot];
    if ((slot->header.sample_count > 0 &&
         (now_us - slot->header.first_monotonic_us) >= live_runtime.publish_cadence_us) ||
        (slot->header.payload_bytes + (live_runtime.active_imu_count * sizeof(struct imu_record)) >
         LIVE_IMU_RECORDS_PER_SLOT * sizeof(struct imu_record))) {
        live_publish_slot(&slot->header, LIVE_STREAM_TYPE_IMU, &live_stream_shared.imu_stats);
        live_runtime.current_imu_slot = live_begin_imu_slot(now_us);
        if (live_runtime.current_imu_slot < 0) {
            return true;
        }
        slot = &live_stream_shared.imu_slots[live_runtime.current_imu_slot];
    }

    record_index = slot->header.payload_bytes / sizeof(struct imu_record);
    if (imu_frame.available) {
        imu_sensor_read(&imu_frame, &ax, &ay, &az, &gx, &gy, &gz);
        slot->payload[record_index++] = (struct imu_record){.ax = ax, .ay = ay, .az = az, .gx = gx, .gy = gy, .gz = gz};
    }
    if (imu_fork.available) {
        imu_sensor_read(&imu_fork, &ax, &ay, &az, &gx, &gy, &gz);
        slot->payload[record_index++] = (struct imu_record){.ax = ax, .ay = ay, .az = az, .gx = gx, .gy = gy, .gz = gz};
    }
    if (imu_rear.available) {
        imu_sensor_read(&imu_rear, &ax, &ay, &az, &gx, &gy, &gz);
        slot->payload[record_index++] = (struct imu_record){.ax = ax, .ay = ay, .az = az, .gx = gx, .gy = gy, .gz = gz};
    }

    slot->header.sample_count++;
    slot->header.payload_bytes = record_index * sizeof(struct imu_record);
    live_stream_shared.imu_stats.next_index++;

    return true;
}
#endif

#if HAS_GPS
static bool live_gps_timer_cb(repeating_timer_t *timer) {
    if (!live_runtime.active || !live_runtime.gps_enabled) {
        return false;
    }

    if (gps.available) {
        gps.process(&gps);
    }

    return true;
}
#endif

static uint32_t live_requested_rate_or_default(uint32_t requested_rate, uint32_t default_rate, uint32_t max_rate) {
    uint32_t resolved_rate = requested_rate == 0 ? default_rate : requested_rate;
    if (resolved_rate == 0) {
        resolved_rate = 1;
    }
    if (resolved_rate > max_rate) {
        resolved_rate = max_rate;
    }
    return resolved_rate;
}

bool live_stream_core0_start(const struct live_start_request *req, struct live_start_response *resp) {
    uint32_t requested_mask;

    memset(resp, 0, sizeof(*resp));

    if (live_runtime.active) {
        resp->result = LIVE_START_RESULT_BUSY;
        return false;
    }

    requested_mask = req->sensor_mask;
    if (requested_mask == 0 || req->protocol_version != LIVE_PROTOCOL_VERSION) {
        resp->result = LIVE_START_RESULT_INVALID_REQUEST;
        return false;
    }

    resp->publish_cadence_ms = LIVE_PUBLISH_CADENCE_MS;
    resp->session_id = live_stream_shared_next_session_id();
    resp->session_start_monotonic_us = time_us_64();
    resp->session_start_utc = rtc_timestamp();

    live_stream_shared_reset();

    live_runtime.publish_cadence_us = LIVE_PUBLISH_CADENCE_MS * 1000u;
    live_runtime.current_travel_slot = -1;
    live_runtime.current_imu_slot = -1;
    live_runtime.current_gps_slot = -1;

    if ((requested_mask & LIVE_SENSOR_MASK_TRAVEL) != 0u) {
        if (!fork_sensor.available && !shock_sensor.available) {
            resp->result = LIVE_START_RESULT_UNAVAILABLE;
            return false;
        }

        resp->accepted_travel_hz =
            live_requested_rate_or_default(req->requested_travel_hz, TRAVEL_SAMPLE_RATE, LIVE_MAX_TRAVEL_HZ);
        resp->travel_period_us = 1000000u / resp->accepted_travel_hz;
        resp->accepted_travel_hz = 1000000u / resp->travel_period_us;
        resp->selected_sensor_mask |= LIVE_SENSOR_MASK_TRAVEL;
        live_runtime.travel_period_us = resp->travel_period_us;
    }

#if HAS_IMU
    if ((requested_mask & LIVE_SENSOR_MASK_IMU) != 0u) {
        if (imu_frame.available) {
            resp->active_imu_count++;
            resp->active_imu_mask |= 0x01u;
        }
        if (imu_fork.available) {
            resp->active_imu_count++;
            resp->active_imu_mask |= 0x02u;
        }
        if (imu_rear.available) {
            resp->active_imu_count++;
            resp->active_imu_mask |= 0x04u;
        }

        if (resp->active_imu_count == 0u) {
            resp->result = LIVE_START_RESULT_UNAVAILABLE;
            return false;
        }

        resp->accepted_imu_hz = live_requested_rate_or_default(req->requested_imu_hz, IMU_SAMPLE_RATE, LIVE_MAX_IMU_HZ);
        resp->imu_period_us = 1000000u / resp->accepted_imu_hz;
        resp->accepted_imu_hz = 1000000u / resp->imu_period_us;
        resp->selected_sensor_mask |= LIVE_SENSOR_MASK_IMU;
        live_runtime.imu_period_us = resp->imu_period_us;
        live_runtime.active_imu_count = resp->active_imu_count;
        live_runtime.active_imu_mask = resp->active_imu_mask;
    }
#endif

#if HAS_GPS
    if ((requested_mask & LIVE_SENSOR_MASK_GPS) != 0u) {
        if (!gps.available) {
            resp->result = LIVE_START_RESULT_UNAVAILABLE;
            return false;
        }

        resp->accepted_gps_fix_hz =
            live_requested_rate_or_default(req->requested_gps_fix_hz, GPS_SAMPLE_RATE, LIVE_MAX_GPS_FIX_HZ);
        resp->gps_fix_interval_ms = 1000u / resp->accepted_gps_fix_hz;
        if (resp->gps_fix_interval_ms == 0u) {
            resp->gps_fix_interval_ms = 1u;
        }
        resp->accepted_gps_fix_hz = 1000u / resp->gps_fix_interval_ms;

        if (!gps.power_on(&gps)) {
            resp->result = LIVE_START_RESULT_INTERNAL_ERROR;
            return false;
        }

        if (!gps_sensor_configure(&gps, (uint16_t)resp->gps_fix_interval_ms, true, true, true, true, false)) {
            gps.power_off(&gps);
            resp->result = LIVE_START_RESULT_INTERNAL_ERROR;
            return false;
        }

        if (!add_repeating_timer_us(LIVE_GPS_RX_DRAIN_INTERVAL_US, live_gps_timer_cb, NULL, &live_runtime.gps_timer)) {
            gps.power_off(&gps);
            resp->result = LIVE_START_RESULT_INTERNAL_ERROR;
            return false;
        }

        resp->selected_sensor_mask |= LIVE_SENSOR_MASK_GPS;
        live_runtime.gps_enabled = true;
        live_runtime.gps_fix_interval_ms = resp->gps_fix_interval_ms;
    }
#endif

    if (resp->selected_sensor_mask == 0u) {
        resp->result = LIVE_START_RESULT_INVALID_REQUEST;
        return false;
    }

    if ((resp->selected_sensor_mask & LIVE_SENSOR_MASK_TRAVEL) != 0u &&
        !add_repeating_timer_us(-(int64_t)live_runtime.travel_period_us, live_travel_cb, NULL,
                                &live_runtime.travel_timer)) {
        live_stream_core0_stop();
        resp->result = LIVE_START_RESULT_INTERNAL_ERROR;
        return false;
    }
    if ((resp->selected_sensor_mask & LIVE_SENSOR_MASK_TRAVEL) != 0u) {
        live_runtime.travel_enabled = true;
    }

#if HAS_IMU
    if ((resp->selected_sensor_mask & LIVE_SENSOR_MASK_IMU) != 0u &&
        !add_repeating_timer_us(-(int64_t)live_runtime.imu_period_us, live_imu_cb, NULL, &live_runtime.imu_timer)) {
        live_stream_core0_stop();
        resp->result = LIVE_START_RESULT_INTERNAL_ERROR;
        return false;
    }
    if ((resp->selected_sensor_mask & LIVE_SENSOR_MASK_IMU) != 0u) {
        live_runtime.imu_enabled = true;
    }
#endif

    live_runtime.active = true;
    live_stream_shared.active = true;
    live_stream_shared.selected_sensor_mask = resp->selected_sensor_mask;
    live_stream_shared.active_imu_mask = resp->active_imu_mask;
    live_stream_shared.start_response = *resp;
    resp->result = LIVE_START_RESULT_OK;
    live_stream_shared.start_response.result = LIVE_START_RESULT_OK;

    return true;
}

void live_stream_core0_stop(void) {
    struct live_slot_header *header;

    if (!live_runtime.active && !live_stream_shared.active && !live_runtime.travel_enabled &&
        !live_runtime.imu_enabled && !live_runtime.gps_enabled) {
        return;
    }

    if (live_runtime.travel_enabled) {
        cancel_repeating_timer(&live_runtime.travel_timer);
        if (live_runtime.current_travel_slot >= 0) {
            header = &live_stream_shared.travel_slots[live_runtime.current_travel_slot].header;
            live_publish_slot(header, LIVE_STREAM_TYPE_TRAVEL, &live_stream_shared.travel_stats);
            live_runtime.current_travel_slot = -1;
        }
    }

#if HAS_IMU
    if (live_runtime.imu_enabled) {
        cancel_repeating_timer(&live_runtime.imu_timer);
        if (live_runtime.current_imu_slot >= 0) {
            header = &live_stream_shared.imu_slots[live_runtime.current_imu_slot].header;
            live_publish_slot(header, LIVE_STREAM_TYPE_IMU, &live_stream_shared.imu_stats);
            live_runtime.current_imu_slot = -1;
        }
    }
#endif

#if HAS_GPS
    if (live_runtime.gps_enabled) {
        cancel_repeating_timer(&live_runtime.gps_timer);
        if (live_runtime.current_gps_slot >= 0) {
            header = &live_stream_shared.gps_slots[live_runtime.current_gps_slot].header;
            live_publish_slot(header, LIVE_STREAM_TYPE_GPS, &live_stream_shared.gps_stats);
            live_runtime.current_gps_slot = -1;
        }
        gps.power_off(&gps);
    }
#endif

    memset(&live_runtime, 0, sizeof(live_runtime));
    live_runtime.current_travel_slot = -1;
    live_runtime.current_imu_slot = -1;
    live_runtime.current_gps_slot = -1;
    live_stream_shared.active = false;
}

bool live_stream_core0_active(void) { return live_runtime.active; }

void live_stream_core0_service(void) {
    switch (live_stream_get_control_state()) {
        case LIVE_CONTROL_START_REQUESTED:
            live_stream_core0_start(&live_stream_shared.start_request, &live_stream_shared.start_response);
            live_stream_set_control_state(LIVE_CONTROL_START_RESPONSE_READY);
            break;
        case LIVE_CONTROL_STOP_REQUESTED:
            live_stream_core0_stop();
            live_stream_set_control_state(LIVE_CONTROL_STOP_RESPONSE_READY);
            break;
        default:
            break;
    }
}

#if HAS_GPS
void live_stream_core0_on_gps_fix(const struct gps_telemetry *telemetry) {
    uint64_t now_us;
    struct live_gps_slot *slot;

    if (!live_runtime.active || !live_runtime.gps_enabled) {
        return;
    }

    now_us = time_us_64();
    if (live_runtime.current_gps_slot < 0) {
        live_runtime.current_gps_slot = live_begin_gps_slot(now_us);
        if (live_runtime.current_gps_slot < 0) {
            return;
        }
    }

    slot = &live_stream_shared.gps_slots[live_runtime.current_gps_slot];
    if ((slot->header.sample_count > 0 &&
         (now_us - slot->header.first_monotonic_us) >= live_runtime.publish_cadence_us) ||
        slot->header.sample_count >= LIVE_GPS_SAMPLES_PER_SLOT) {
        live_publish_slot(&slot->header, LIVE_STREAM_TYPE_GPS, &live_stream_shared.gps_stats);
        live_runtime.current_gps_slot = live_begin_gps_slot(now_us);
        if (live_runtime.current_gps_slot < 0) {
            return;
        }
        slot = &live_stream_shared.gps_slots[live_runtime.current_gps_slot];
    }

    slot->payload[slot->header.sample_count] = (struct gps_record){
        .date = telemetry->date,
        .time_ms = telemetry->time_ms,
        .latitude = telemetry->latitude,
        .longitude = telemetry->longitude,
        .altitude = telemetry->altitude,
        .speed = telemetry->speed,
        .heading = telemetry->heading,
        .fix_mode = (uint8_t)telemetry->fix_mode,
        .satellites = telemetry->satellites,
        .epe_2d = telemetry->epe_2d,
        .epe_3d = telemetry->epe_3d,
    };
    slot->header.sample_count++;
    slot->header.payload_bytes = slot->header.sample_count * sizeof(struct gps_record);
    live_stream_shared.gps_stats.next_index++;
}

void gps_fix_router_on_fix(const struct gps_telemetry *telemetry) {
    if (live_stream_core0_active()) {
        live_stream_core0_on_gps_fix(telemetry);
    } else {
        recording_on_gps_fix(telemetry);
    }
}
#endif