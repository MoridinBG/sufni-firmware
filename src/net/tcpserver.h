#ifndef _TCPSERVER_H
#define _TCPSERVER_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "live_core1_protocol.h"
#include "management_protocol.h"

#include "lwip/tcp.h"
#include "pico/cyw43_arch.h"

#define TCPSERVER_RX_BUFFER_SIZE MANAGEMENT_PROTOCOL_MAX_RX_FRAME_SIZE

enum tcpserver_protocol_mode {
    TCPSERVER_PROTOCOL_UNKNOWN = 0,
    TCPSERVER_PROTOCOL_LIVE = 1,
    TCPSERVER_PROTOCOL_MANAGEMENT = 2,
};

struct tcpserver_protocol_ops {
    bool (*can_accept)(const struct tcpserver *server);
    void (*on_accept)(struct tcpserver *server);
    void (*on_disconnect)(struct tcpserver *server);
    bool (*detect)(const struct tcpserver *server);
    bool (*process_rx)(struct tcpserver *server);
    void (*service)(struct tcpserver *server);
    bool (*needs_service)(const struct tcpserver *server);
};

struct tcpserver_options {
    bool allow_live_preview;
    bool enable_mdns;
    bool mark_downloaded_on_success;
};

struct tcpserver {
    struct tcp_pcb *server_pcb;
    struct tcp_pcb *client_pcb;
    struct netif *netif;
    s8_t mdns_slot;
    bool mdns_initialized;
    bool mdns_netif_added;
    bool mdns_service_added;
    bool finish_requested;
    bool client_connected;
    bool close_client_requested;
    volatile bool err_disconnect_pending;
    int last_error;
    enum tcpserver_protocol_mode protocol_mode;
    const struct tcpserver_protocol_ops *protocol_ops;
    uint16_t rx_len;
    struct tcpserver_options options;
    uint8_t rx_buffer[TCPSERVER_RX_BUFFER_SIZE];
    struct live_protocol_session live;
    struct management_session management;
};

static inline void tcpserver_consume_rx(struct tcpserver *server, uint16_t bytes_to_consume) {
    if (bytes_to_consume >= server->rx_len) {
        server->rx_len = 0;
        return;
    }

    memmove(server->rx_buffer, server->rx_buffer + bytes_to_consume, server->rx_len - bytes_to_consume);
    server->rx_len -= bytes_to_consume;
}

bool tcpserver_init(struct tcpserver *server, const struct tcpserver_options *options);
bool tcpserver_run(struct tcpserver *server, volatile bool *stop_requested);
void tcpserver_finish(struct tcpserver *server);

#endif /* _TCPSERVER_H */
