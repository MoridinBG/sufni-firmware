#ifndef LIVE_STREAM_SHARED_H
#define LIVE_STREAM_SHARED_H

#include <stdbool.h>
#include <stdint.h>

#include "pico.h"

#include "sst.h"

#define LIVE_PUBLISH_CADENCE_MS 32u

#define LIVE_MAX_TRAVEL_HZ  1000u
#define LIVE_MAX_IMU_HZ     1000u
#define LIVE_MAX_GPS_FIX_HZ 10u

#define LIVE_TRAVEL_SLOT_COUNT 6u
#define LIVE_IMU_SLOT_COUNT    6u
#define LIVE_GPS_SLOT_COUNT    4u

#define LIVE_MAX_ACTIVE_IMUS         3u
#define LIVE_TRAVEL_SAMPLES_PER_SLOT 32u
#define LIVE_IMU_RECORDS_PER_SLOT    96u
#define LIVE_GPS_SAMPLES_PER_SLOT    16u

enum live_stream_type {
    LIVE_STREAM_TYPE_TRAVEL = 1,
    LIVE_STREAM_TYPE_IMU = 2,
    LIVE_STREAM_TYPE_GPS = 3,
};

enum live_slot_state {
    LIVE_SLOT_FREE = 0,
    LIVE_SLOT_FILLING = 1,
    LIVE_SLOT_READY = 2,
    LIVE_SLOT_SENDING = 3,
};

enum live_control_state {
    LIVE_CONTROL_IDLE = 0,
    LIVE_CONTROL_START_REQUESTED = 1,
    LIVE_CONTROL_START_RESPONSE_READY = 2,
    LIVE_CONTROL_STOP_REQUESTED = 3,
    LIVE_CONTROL_STOP_RESPONSE_READY = 4,
};

enum live_start_result {
    LIVE_START_RESULT_OK = 0,
    LIVE_START_RESULT_INVALID_REQUEST = -1,
    LIVE_START_RESULT_BUSY = -2,
    LIVE_START_RESULT_UNAVAILABLE = -3,
    LIVE_START_RESULT_INTERNAL_ERROR = -4,
};

struct live_start_request {
    uint32_t protocol_version;
    uint32_t sensor_mask;
    uint32_t requested_travel_hz;
    uint32_t requested_imu_hz;
    uint32_t requested_gps_fix_hz;
};

struct live_start_response {
    int32_t result;
    uint32_t session_id;
    uint32_t selected_sensor_mask;
    uint32_t accepted_travel_hz;
    uint32_t accepted_imu_hz;
    uint32_t accepted_gps_fix_hz;
    uint32_t travel_period_us;
    uint32_t imu_period_us;
    uint32_t gps_fix_interval_ms;
    uint32_t publish_cadence_ms;
    uint32_t active_imu_count;
    uint32_t active_imu_mask;
    int64_t session_start_utc;
    uint64_t session_start_monotonic_us;
};

struct live_stream_stats {
    volatile uint32_t queue_depth;
    volatile uint32_t dropped_batches;
    volatile uint32_t next_sequence;
    volatile uint64_t next_index;
};

struct live_slot_header {
    volatile uint32_t state;
    uint32_t sequence;
    uint64_t first_index;
    uint64_t first_monotonic_us;
    uint32_t sample_count;
    uint32_t payload_bytes;
};

struct live_travel_slot {
    struct live_slot_header header;
    struct travel_record payload[LIVE_TRAVEL_SAMPLES_PER_SLOT];
};

struct live_imu_slot {
    struct live_slot_header header;
    struct imu_record payload[LIVE_IMU_RECORDS_PER_SLOT];
};

struct live_gps_slot {
    struct live_slot_header header;
    struct gps_record payload[LIVE_GPS_SAMPLES_PER_SLOT];
};

struct live_stream_shared_state {
    volatile uint32_t control_state;
    volatile bool active;
    volatile uint32_t selected_sensor_mask;
    volatile uint32_t active_imu_mask;

    struct live_start_request start_request;
    struct live_start_response start_response;

    struct live_stream_stats travel_stats;
    struct live_stream_stats imu_stats;
    struct live_stream_stats gps_stats;

    struct live_travel_slot travel_slots[LIVE_TRAVEL_SLOT_COUNT];
    struct live_imu_slot imu_slots[LIVE_IMU_SLOT_COUNT];
    struct live_gps_slot gps_slots[LIVE_GPS_SLOT_COUNT];
};

extern struct live_stream_shared_state live_stream_shared;

void live_stream_shared_reset(void);
uint32_t live_stream_shared_next_session_id(void);
uint32_t live_stream_shared_ready_depth(enum live_stream_type stream_type);

static inline void live_stream_set_control_state(enum live_control_state state) {
    __dmb();
    live_stream_shared.control_state = (uint32_t)state;
}

static inline enum live_control_state live_stream_get_control_state(void) {
    enum live_control_state state = (enum live_control_state)live_stream_shared.control_state;
    __dmb();
    return state;
}

static inline void live_stream_publish_slot(volatile uint32_t *state) {
    __dmb();
    *state = LIVE_SLOT_READY;
}

static inline void live_stream_release_slot(volatile uint32_t *state) {
    __dmb();
    *state = LIVE_SLOT_FREE;
}

#endif // LIVE_STREAM_SHARED_H