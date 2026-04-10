#ifndef _TCPSERVER_H
#define _TCPSERVER_H

#include <stdint.h>

#include "lwip/tcp.h"
#include "pico/cyw43_arch.h"

#define TCPSERVER_RX_BUFFER_SIZE 512u

enum tcpserver_protocol_mode {
    TCPSERVER_PROTOCOL_UNKNOWN = 0,
    TCPSERVER_PROTOCOL_LEGACY = 1,
    TCPSERVER_PROTOCOL_LIVE = 2,
};

struct tcpserver {
    struct tcp_pcb *server_pcb;
    struct tcp_pcb *client_pcb;
    int status;
    int requested_file;
    int data_len;
    int sent_len;
    s8_t mdns_slot;
    bool mdns_initialized;
    uint8_t protocol_mode;
    bool live_session_active;
    bool live_start_pending;
    bool live_stop_pending;
    uint32_t live_tx_sequence;
    uint64_t last_stats_us;
    uint16_t rx_len;
    uint8_t rx_buffer[TCPSERVER_RX_BUFFER_SIZE];
};

bool tcpserver_init(struct tcpserver *server);
bool tcpserver_run(struct tcpserver *server, volatile bool *stop_requested);
void tcpserver_finish(struct tcpserver *server);

#endif /* _TCPSERVER_H */
