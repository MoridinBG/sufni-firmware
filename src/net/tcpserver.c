#include "tcpserver.h"

#include "lwip/apps/mdns.h"
#include "lwip/netif.h"
#include "lwip/tcp.h"
#include "pico/time.h"
#include "pico/unique_id.h"
#include "wifi.h"

#include <stdio.h>
#include <string.h>

#include "../util/log.h"

#define TCP_PORT    1557
#define POLL_TIME_S 5

struct tcpserver_protocol_registration {
    enum tcpserver_protocol_mode mode;
    const struct tcpserver_protocol_ops *ops;
};

static const struct tcpserver_protocol_registration tcpserver_protocols[] = {
    {TCPSERVER_PROTOCOL_LIVE, &live_core1_protocol_ops},
    {TCPSERVER_PROTOCOL_MANAGEMENT, &management_protocol_ops},
};

static void mdns_srv_txt(struct mdns_service *service, void *txt_userdata) {
    LWIP_UNUSED_ARG(txt_userdata);

    pico_unique_board_id_t board_id;
    pico_get_unique_board_id(&board_id);

    char txt[4 + PICO_UNIQUE_BOARD_ID_SIZE_BYTES * 2 + 1]; // "bid=" + hex + null
    memcpy(txt, "bid=", 4);
    for (int i = 0; i < PICO_UNIQUE_BOARD_ID_SIZE_BYTES; i++) {
        snprintf(txt + 4 + i * 2, 3, "%02x", board_id.id[i]);
    }

    err_t res = mdns_resp_add_service_txtitem(service, txt, (uint8_t)strlen(txt));
    LWIP_ERROR("mdns add service txt failed\n", (res == ERR_OK), return);
}

static void tcpserver_reset_connection_state(struct tcpserver *server) {
    server->client_connected = false;
    server->close_client_requested = false;
    server->protocol_mode = TCPSERVER_PROTOCOL_UNKNOWN;
    server->protocol_ops = NULL;
    server->rx_len = 0;
    memset(server->rx_buffer, 0, sizeof(server->rx_buffer));
    server->live = (struct live_protocol_session){0};
    management_protocol_reset_session(server);
}

static bool tcpserver_can_accept_client(const struct tcpserver *server) {
    size_t index;

    if (server->err_disconnect_pending || server->client_connected) {
        return false;
    }

    for (index = 0; index < (sizeof(tcpserver_protocols) / sizeof(tcpserver_protocols[0])); ++index) {
        const struct tcpserver_protocol_registration *registration = &tcpserver_protocols[index];

        if (registration->mode == TCPSERVER_PROTOCOL_LIVE && !server->options.allow_live_preview) {
            continue;
        }
        if (!registration->ops->can_accept(server)) {
            return false;
        }
    }

    return true;
}

// Finish a disconnect that tcp_server_err could only flag (ISR context).
static void tcpserver_complete_error_disconnect(struct tcpserver *server) {
    if (!server->err_disconnect_pending || server->client_pcb != NULL) {
        return;
    }

    // Keep err_disconnect_pending true during cleanup so tcp_server_accept
    // rejects new connections until the protocol disconnect is fully handled.
    if (server->protocol_ops != NULL) {
        server->protocol_ops->on_disconnect(server);
    }
    tcpserver_reset_connection_state(server);
    server->err_disconnect_pending = false;

    if (server->last_error != 0) {
        LOG("TCP", "Client disconnected with error: %d\n", server->last_error);
        server->last_error = 0;
    }
}

static err_t tcpserver_close_client(struct tcpserver *server) {
    err_t err = ERR_OK;

    if (server->client_pcb != NULL) {
        struct tcp_pcb *pcb = server->client_pcb;

        if (server->protocol_ops != NULL) {
            server->protocol_ops->on_disconnect(server);
        }

        // All lwIP PCB operations must be inside the lock — the CYW43 background
        // interrupt can otherwise process packets mid-close and corrupt the pool.
        // Re-check the pointer: tcp_server_err may have freed the PCB during
        // on_disconnect (e.g. live abort spin-wait).
        cyw43_arch_lwip_begin();
        if (server->client_pcb == pcb) {
            tcp_arg(pcb, NULL);
            tcp_poll(pcb, NULL, 0);
            tcp_sent(pcb, NULL);
            tcp_recv(pcb, NULL);
            tcp_err(pcb, NULL);
            err = tcp_close(pcb);
            if (err != ERR_OK) {
                tcp_abort(pcb);
                err = ERR_ABRT;
            }
            server->client_pcb = NULL;
        }
        cyw43_arch_lwip_end();
    }

    // Clear the flag in case tcp_server_err raced with this close path.
    server->err_disconnect_pending = false;
    tcpserver_reset_connection_state(server);
    return err;
}

// Match the first 4 RX bytes against registered protocol magics.
// No-op once protocol_ops is set. Returns false to reject the connection.
static bool tcpserver_select_protocol(struct tcpserver *server) {
    size_t index;

    if (server->protocol_ops != NULL) {
        return true;
    }

    for (index = 0; index < (sizeof(tcpserver_protocols) / sizeof(tcpserver_protocols[0])); ++index) {
        const struct tcpserver_protocol_registration *registration = &tcpserver_protocols[index];

        if (registration->mode == TCPSERVER_PROTOCOL_LIVE && !server->options.allow_live_preview) {
            continue;
        }
        if (!registration->ops->detect(server)) {
            continue;
        }

        server->protocol_mode = registration->mode;
        server->protocol_ops = registration->ops;
        server->protocol_ops->on_accept(server);
        LOG("TCP", "Protocol selected: %s\n", registration->mode == TCPSERVER_PROTOCOL_LIVE ? "LIVE" : "MGMT");
        return true;
    }

    if (server->rx_len >= sizeof(uint32_t)) {
        LOG("TCP", "Rejecting unknown initial traffic\n");
        server->last_error = PICO_ERROR_INVALID_DATA;
        return false;
    }

    return true;
}

static bool tcpserver_process_client(struct tcpserver *server) {
    if (!tcpserver_select_protocol(server)) {
        return false;
    }

    if (server->protocol_ops == NULL) {
        return true;
    }

    if (server->rx_len > 0u && !server->protocol_ops->process_rx(server)) {
        return false;
    }

    if (server->protocol_ops->needs_service(server)) {
        server->protocol_ops->service(server);
    }

    return !server->close_client_requested;
}

static bool tcpserver_consume_live_ack_wakeup(struct tcpserver *server) {
    if (server == NULL || !server->client_connected || server->protocol_mode != TCPSERVER_PROTOCOL_LIVE) {
        return false;
    }

    return __atomic_exchange_n(&server->live.tx_ack_wakeup_pending, false, __ATOMIC_ACQ_REL);
}

static err_t tcp_server_sent(void *arg, struct tcp_pcb *tpcb, u16_t len) {
    struct tcpserver *server = (struct tcpserver *)arg;

    LWIP_UNUSED_ARG(tpcb);

    if (server != NULL && server->protocol_mode == TCPSERVER_PROTOCOL_LIVE) {
        server->live.tx_ack_events++;
        server->live.tx_acked_bytes += len;
        server->live.tx_last_ack_len = len;
        server->live.tx_last_ack_us = time_us_64();
        __atomic_store_n(&server->live.tx_ack_wakeup_pending, true, __ATOMIC_RELEASE);
    }

    return ERR_OK;
}

static err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
    struct tcpserver *server = (struct tcpserver *)arg;

    if (p == NULL) {
        server->close_client_requested = true;
        server->last_error = 0;
        return ERR_OK;
    }

    cyw43_arch_lwip_check();
    if (err != ERR_OK) {
        server->close_client_requested = true;
        server->last_error = err;
        pbuf_free(p);
        return ERR_OK;
    }

    if (p->tot_len > 0u) {
        tcp_recved(tpcb, p->tot_len);

        if ((server->rx_len + p->tot_len) > TCPSERVER_RX_BUFFER_SIZE) {
            server->close_client_requested = true;
            server->last_error = PICO_ERROR_GENERIC;
        } else {
            pbuf_copy_partial(p, server->rx_buffer + server->rx_len, p->tot_len, 0);
            server->rx_len += p->tot_len;
        }
    }

    pbuf_free(p);
    return ERR_OK;
}

static err_t tcp_server_poll(void *arg, struct tcp_pcb *tpcb) {
    LWIP_UNUSED_ARG(arg);
    LWIP_UNUSED_ARG(tpcb);
    return ERR_OK;
}

static void tcp_server_err(void *arg, err_t err) {
    struct tcpserver *server = (struct tcpserver *)arg;

    if (server == NULL) {
        return;
    }

    // PCB is already freed by lwip — just record the error and flag cleanup.
    // Full disconnect (including live abort + quiescence wait) is deferred to
    // tcpserver_run where it runs on Core 1 and can safely block on Core 0.
    server->client_pcb = NULL;
    server->client_connected = false;
    server->last_error = err == ERR_ABRT ? 0 : err;
    server->err_disconnect_pending = true;
}

static err_t tcp_server_accept(void *arg, struct tcp_pcb *client_pcb, err_t err) {
    struct tcpserver *server = (struct tcpserver *)arg;

    if (err != ERR_OK || client_pcb == NULL) {
        LOG("TCP", "Client connection failed: %d\n", err);
        return ERR_VAL;
    }

    if (!tcpserver_can_accept_client(server)) {
        LOG("TCP", "Rejecting client while protocol state is busy\n");
        tcp_abort(client_pcb);
        return ERR_ABRT;
    }

    // Safety net: detach and abort any orphaned PCB left by a previous accept
    // that raced with a disconnect (shouldn't happen with the client_connected
    // guard above, but prevents a leaked PCB from corrupting state via callbacks).
    if (server->client_pcb != NULL) {
        tcp_arg(server->client_pcb, NULL);
        tcp_poll(server->client_pcb, NULL, 0);
        tcp_sent(server->client_pcb, NULL);
        tcp_recv(server->client_pcb, NULL);
        tcp_err(server->client_pcb, NULL);
        tcp_abort(server->client_pcb);
        server->client_pcb = NULL;
    }

    LOG("TCP", "Client connected\n");
    tcpserver_reset_connection_state(server);
    server->client_pcb = client_pcb;
    server->client_connected = true;

    tcp_arg(client_pcb, server);
    tcp_sent(client_pcb, tcp_server_sent);
    tcp_recv(client_pcb, tcp_server_recv);
    tcp_poll(client_pcb, tcp_server_poll, POLL_TIME_S * 2);
    tcp_err(client_pcb, tcp_server_err);
    return ERR_OK;
}

static bool tcp_server_open(struct tcpserver *server) {
    struct tcp_pcb *pcb;
    err_t err;

    cyw43_arch_lwip_begin();
    pcb = tcp_new_ip_type(IPADDR_TYPE_ANY);
    if (pcb == NULL) {
        cyw43_arch_lwip_end();
        return false;
    }

    err = tcp_bind(pcb, NULL, TCP_PORT);
    if (err != ERR_OK) {
        tcp_close(pcb);
        cyw43_arch_lwip_end();
        return false;
    }

    server->server_pcb = tcp_listen_with_backlog(pcb, 1);
    if (server->server_pcb == NULL) {
        tcp_close(pcb);
        cyw43_arch_lwip_end();
        return false;
    }

    tcp_arg(server->server_pcb, server);
    tcp_accept(server->server_pcb, tcp_server_accept);
    cyw43_arch_lwip_end();
    return true;
}

static void tcpserver_teardown(struct tcpserver *server) {
    tcpserver_complete_error_disconnect(server);

    if (server->client_pcb != NULL) {
        tcpserver_close_client(server);
    }

    cyw43_arch_lwip_begin();
    if (server->server_pcb != NULL) {
        tcp_arg(server->server_pcb, NULL);
        tcp_close(server->server_pcb);
        server->server_pcb = NULL;
    }

    if (server->mdns_service_added && server->netif != NULL) {
        mdns_resp_del_service(server->netif, server->mdns_slot);
        server->mdns_service_added = false;
    }
    if (server->mdns_netif_added && server->netif != NULL) {
        mdns_resp_remove_netif(server->netif);
        server->mdns_netif_added = false;
    }
    cyw43_arch_lwip_end();
}

bool tcpserver_init(struct tcpserver *server, const struct tcpserver_options *options) {
    LOG("TCP", "Initializing server\n");

    server->server_pcb = NULL;
    server->client_pcb = NULL;
    server->netif = wifi_active_netif();
    server->mdns_slot = -1;
    server->mdns_netif_added = false;
    server->mdns_service_added = false;
    server->finish_requested = false;
    server->err_disconnect_pending = false;
    server->last_error = 0;
    server->options = options != NULL ? *options
                                      : (struct tcpserver_options){
                                            .allow_live_preview = true,
                                            .enable_mdns = true,
                                        };
    tcpserver_reset_connection_state(server);

    if (server->netif == NULL) {
        LOG("TCP", "No active netif available for TCP server\n");
        return false;
    }

    if (server->options.enable_mdns) {
        if (server->mdns_initialized) {
            mdns_resp_restart(server->netif);
        } else {
            mdns_resp_init();
            server->mdns_initialized = true;
        }

        if (mdns_resp_add_netif(server->netif, "sufni_telemetry_daq") == ERR_OK) {
            server->mdns_netif_added = true;
            server->mdns_slot = mdns_resp_add_service(server->netif, "sufnidaq", "_gosst", DNSSD_PROTO_TCP, TCP_PORT,
                                                      mdns_srv_txt, NULL);
            if (server->mdns_slot >= 0) {
                server->mdns_service_added = true;
                mdns_resp_announce(server->netif);
            } else {
                LOG("TCP", "mDNS service registration failed\n");
            }
        } else {
            LOG("TCP", "mDNS netif registration failed\n");
        }
    }

    if (!tcp_server_open(server)) {
        LOG("TCP", "Failed to open server\n");
        tcpserver_teardown(server);
        return false;
    }

    LOG("TCP", "Server listening on port %d, IP: %s\n", TCP_PORT, ip4addr_ntoa(netif_ip4_addr(server->netif)));
    return true;
}

bool tcpserver_run(struct tcpserver *server, tcpserver_stop_requested_fn stop_requested, void *stop_context) {
    LOG("TCP", "Server started, waiting for requests\n");

    while (!server->finish_requested) {
        bool skip_sleep = false;

        if (stop_requested != NULL && stop_requested(stop_context)) {
            tcpserver_finish(server);
        }

        tcpserver_complete_error_disconnect(server);

        if (server->client_connected) {
            if (server->close_client_requested || !tcpserver_process_client(server)) {
                int error_code = server->last_error;

                tcpserver_close_client(server);
                if (error_code != 0) {
                    LOG("TCP", "Client/session closed with error: %d\n", error_code);
                }
                server->last_error = 0;
            }
        }

        skip_sleep = tcpserver_consume_live_ack_wakeup(server);
        if (!skip_sleep) {
            sleep_ms(1);
        }
    }

    LOG("TCP", "Server stopping\n");
    tcpserver_teardown(server);
    return true;
}

void tcpserver_finish(struct tcpserver *server) {
    LOG("TCP", "Server finish requested\n");
    server->finish_requested = true;
    server->close_client_requested = true;
}