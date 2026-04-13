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

struct live_protocol_session {
    enum live_session_phase phase;
    enum live_handshake_step handshake_step;
    bool stop_queued;
    uint32_t tx_sequence;
    uint64_t last_stats_us;
};

extern const struct tcpserver_protocol_ops live_core1_protocol_ops;

#endif // LIVE_CORE1_PROTOCOL_H
