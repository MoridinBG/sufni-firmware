#ifndef LIVE_PROTOCOL_H
#define LIVE_PROTOCOL_H

#include <stdint.h>

#define LIVE_PROTOCOL_MAGIC   0x4556494Cu
#define LIVE_PROTOCOL_VERSION 1u

#define LIVE_SENSOR_MASK_TRAVEL 0x01u
#define LIVE_SENSOR_MASK_IMU    0x02u
#define LIVE_SENSOR_MASK_GPS    0x04u

#define LIVE_SESSION_FLAG_CALIBRATED_ONLY                   0x01u
#define LIVE_SESSION_FLAG_MUTUALLY_EXCLUSIVE_WITH_RECORDING 0x02u

enum live_frame_type {
    LIVE_FRAME_START_LIVE = 1,
    LIVE_FRAME_STOP_LIVE = 2,
    LIVE_FRAME_PING = 3,
    LIVE_FRAME_START_LIVE_ACK = 16,
    LIVE_FRAME_STOP_LIVE_ACK = 17,
    LIVE_FRAME_ERROR = 18,
    LIVE_FRAME_PONG = 19,
    LIVE_FRAME_SESSION_HEADER = 20,
    LIVE_FRAME_TRAVEL_BATCH = 32,
    LIVE_FRAME_IMU_BATCH = 33,
    LIVE_FRAME_GPS_BATCH = 34,
    LIVE_FRAME_SESSION_STATS = 48,
};

struct live_frame_header {
    uint32_t magic;
    uint16_t version;
    uint16_t frame_type;
    uint32_t payload_length;
    uint32_t sequence;
} __attribute__((packed));

struct live_start_request_frame {
    uint32_t sensor_mask;
    uint32_t travel_hz;
    uint32_t imu_hz;
    uint32_t gps_fix_hz;
} __attribute__((packed));

struct live_start_ack_frame {
    int32_t result;
    uint32_t session_id;
    uint32_t selected_sensor_mask;
} __attribute__((packed));

struct live_stop_ack_frame {
    uint32_t session_id;
} __attribute__((packed));

struct live_error_frame {
    int32_t error_code;
} __attribute__((packed));

struct live_session_header_frame {
    uint32_t session_id;
    uint32_t accepted_travel_hz;
    uint32_t accepted_imu_hz;
    uint32_t accepted_gps_fix_hz;
    int64_t session_start_utc;
    uint64_t session_start_monotonic_us;
    uint32_t active_imu_mask;
    float imu_accel_lsb_per_g[3];
    float imu_gyro_lsb_per_dps[3];
    uint32_t flags;
} __attribute__((packed));

struct live_batch_payload {
    uint32_t session_id;
    uint32_t stream_sequence;
    uint64_t first_index;
    uint64_t first_monotonic_us;
    uint32_t sample_count;
} __attribute__((packed));

struct live_session_stats_frame {
    uint32_t session_id;
    uint32_t travel_queue_depth;
    uint32_t imu_queue_depth;
    uint32_t gps_queue_depth;
    uint32_t travel_dropped_batches;
    uint32_t imu_dropped_batches;
    uint32_t gps_dropped_batches;
} __attribute__((packed));

#endif // LIVE_PROTOCOL_H