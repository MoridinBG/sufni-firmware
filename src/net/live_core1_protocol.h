#ifndef LIVE_CORE1_PROTOCOL_H
#define LIVE_CORE1_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

struct tcpserver_protocol_ops;

enum live_session_phase {
    LIVE_PHASE_IDLE = 0,
    LIVE_PHASE_STARTING,
    LIVE_PHASE_START_HANDSHAKE,
    LIVE_PHASE_ACTIVE,
    LIVE_PHASE_STOPPING,
    LIVE_PHASE_STOP_DEFERRED,
};

enum live_handshake_step {
    LIVE_HANDSHAKE_ACK = 0,
    LIVE_HANDSHAKE_HEADER,
    LIVE_HANDSHAKE_STATS,
    LIVE_HANDSHAKE_DONE,
};

enum live_tx_diag_frame_kind {
    LIVE_TX_DIAG_FRAME_TRAVEL = 0,
    LIVE_TX_DIAG_FRAME_IMU,
    LIVE_TX_DIAG_FRAME_GPS,
    LIVE_TX_DIAG_FRAME_SESSION_STATS,
    LIVE_TX_DIAG_FRAME_CONTROL,
    LIVE_TX_DIAG_FRAME_COUNT,
};

struct live_protocol_session {
    enum live_session_phase phase;
    enum live_handshake_step handshake_step;
    bool stop_queued;
    bool tx_failure_active;
    bool tx_timeout_fired;
    bool tx_timeout_requested_stop;
    bool tx_timeout_requested_close;
    bool tx_diag_summary_logged;
    uint32_t tx_sequence;
    uint64_t last_stats_us;
    uint64_t tx_failure_started_us;
    uint64_t tx_timeout_fired_us;
    uint32_t tx_fail_lwip_lock;
    uint32_t tx_fail_sndbuf;
    uint32_t tx_fail_write_err;
    int8_t tx_last_write_err;
    uint32_t tx_fail_err_mem;
    uint32_t tx_fail_err_conn;
    uint32_t tx_fail_err_abrt;
    uint32_t tx_fail_err_rst;
    uint32_t tx_fail_err_clsd;
    uint32_t tx_fail_err_val;
    uint32_t tx_fail_err_other;
    uint32_t tx_last_sndbuf;
    uint32_t tx_min_sndbuf;
    uint16_t tx_last_sndqueuelen;
    uint16_t tx_max_sndqueuelen;
    uint64_t tx_enqueued_bytes;
    uint64_t tx_acked_bytes;
    uint32_t tx_ack_events;
    uint16_t tx_last_ack_len;
    uint64_t tx_last_ack_us;
    bool tx_ack_wakeup_pending;
    uint32_t tx_ok_count[LIVE_TX_DIAG_FRAME_COUNT];
    uint32_t tx_fail_count[LIVE_TX_DIAG_FRAME_COUNT];
    uint64_t tx_last_ok_us[LIVE_TX_DIAG_FRAME_COUNT];
    uint64_t tx_last_fail_us[LIVE_TX_DIAG_FRAME_COUNT];
    uint32_t tx_ready_depth_last[3];
    uint32_t tx_ready_depth_max[3];
    uint64_t tx_ready_oldest_age_last_us[3];
    uint64_t tx_ready_oldest_age_max_us[3];
};

extern const struct tcpserver_protocol_ops live_core1_protocol_ops;

#endif // LIVE_CORE1_PROTOCOL_H
