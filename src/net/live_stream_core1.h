#ifndef LIVE_STREAM_CORE1_H
#define LIVE_STREAM_CORE1_H

#include <stdbool.h>
#include <stdint.h>

struct tcpserver_protocol_ops;

struct live_protocol_session {
    bool session_active;
    bool start_pending;
    bool stop_pending;
    uint32_t tx_sequence;
    uint64_t last_stats_us;
};

extern const struct tcpserver_protocol_ops live_stream_core1_protocol_ops;

#endif // LIVE_STREAM_CORE1_H