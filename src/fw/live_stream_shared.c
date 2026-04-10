#include "live_stream_shared.h"

#include <string.h>

struct live_stream_shared_state live_stream_shared;

static uint32_t live_stream_session_id = 1;

static void live_stream_reset_slots(void) {
    uint32_t slot_index;

    for (slot_index = 0; slot_index < LIVE_TRAVEL_SLOT_COUNT; ++slot_index) {
        memset(&live_stream_shared.travel_slots[slot_index], 0, sizeof(live_stream_shared.travel_slots[slot_index]));
        live_stream_shared.travel_slots[slot_index].header.state = LIVE_SLOT_FREE;
    }

    for (slot_index = 0; slot_index < LIVE_IMU_SLOT_COUNT; ++slot_index) {
        memset(&live_stream_shared.imu_slots[slot_index], 0, sizeof(live_stream_shared.imu_slots[slot_index]));
        live_stream_shared.imu_slots[slot_index].header.state = LIVE_SLOT_FREE;
    }

    for (slot_index = 0; slot_index < LIVE_GPS_SLOT_COUNT; ++slot_index) {
        memset(&live_stream_shared.gps_slots[slot_index], 0, sizeof(live_stream_shared.gps_slots[slot_index]));
        live_stream_shared.gps_slots[slot_index].header.state = LIVE_SLOT_FREE;
    }
}

void live_stream_shared_reset(void) {
    memset(&live_stream_shared.travel_stats, 0, sizeof(live_stream_shared.travel_stats));
    memset(&live_stream_shared.imu_stats, 0, sizeof(live_stream_shared.imu_stats));
    memset(&live_stream_shared.gps_stats, 0, sizeof(live_stream_shared.gps_stats));
    live_stream_shared.selected_sensor_mask = 0;
    live_stream_shared.active_imu_mask = 0;
    live_stream_shared.active = false;
    live_stream_reset_slots();
}

uint32_t live_stream_shared_next_session_id(void) { return live_stream_session_id++; }

uint32_t live_stream_shared_ready_depth(enum live_stream_type stream_type) {
    uint32_t slot_index;
    uint32_t depth = 0;

    switch (stream_type) {
        case LIVE_STREAM_TYPE_TRAVEL:
            for (slot_index = 0; slot_index < LIVE_TRAVEL_SLOT_COUNT; ++slot_index) {
                if (live_stream_shared.travel_slots[slot_index].header.state == LIVE_SLOT_READY) {
                    ++depth;
                }
            }
            break;
        case LIVE_STREAM_TYPE_IMU:
            for (slot_index = 0; slot_index < LIVE_IMU_SLOT_COUNT; ++slot_index) {
                if (live_stream_shared.imu_slots[slot_index].header.state == LIVE_SLOT_READY) {
                    ++depth;
                }
            }
            break;
        case LIVE_STREAM_TYPE_GPS:
            for (slot_index = 0; slot_index < LIVE_GPS_SLOT_COUNT; ++slot_index) {
                if (live_stream_shared.gps_slots[slot_index].header.state == LIVE_SLOT_READY) {
                    ++depth;
                }
            }
            break;
    }

    return depth;
}