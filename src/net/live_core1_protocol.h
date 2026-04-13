#ifndef LIVE_CORE1_PROTOCOL_H
#define LIVE_CORE1_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

struct tcpserver_protocol_ops;

struct live_protocol_session {
    bool session_active;
    bool start_pending;
    bool start_ack_sent;
    bool start_header_sent;
    bool start_stats_sent;
    bool stop_pending;
    uint32_t tx_sequence;
    uint64_t last_stats_us;
};

extern const struct tcpserver_protocol_ops live_core1_protocol_ops;

#endif // LIVE_CORE1_PROTOCOL_H