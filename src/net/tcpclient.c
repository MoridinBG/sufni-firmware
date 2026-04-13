#include "lwip/dns.h"
#include "lwip/ip_addr.h"
#include "lwip/pbuf.h"
#include "lwip/tcp.h"
#include "lwip/tcpbase.h"
#include "pico/cyw43_arch.h"
#include "pico/time.h"
#include "pico/unique_id.h"

#include "hw_config.h"

#include "../net/tcpclient.h"
#include "../util/config.h"

#define READ_BUF_LEN (10 * 1024)
#define FILENAME_LENGTH                                                                                                \
    10 // filename is always in 00000.SST format,
       // so length is always 10.
#define POLL_TIME_S 5

#define STATUS_INIT      1
#define STATUS_DNS_FOUND 2
#define STATUS_CONNECTED 3
#define STATUS_HEADER_OK 4
#define STATUS_DATA_SENT 5
#define STATUS_SUCCESS   6

static err_t tcpclient_close(void *arg) {
    struct tcpclient_connection *conn = (struct tcpclient_connection *)arg;
    err_t err = ERR_OK;
    if (conn->pcb != NULL) {
        tcp_arg(conn->pcb, NULL);
        tcp_poll(conn->pcb, NULL, 0);
        tcp_sent(conn->pcb, NULL);
        tcp_recv(conn->pcb, NULL);
        tcp_err(conn->pcb, NULL);
        err = tcp_close(conn->pcb);
        if (err != ERR_OK) {
            tcp_abort(conn->pcb);
            err = ERR_ABRT;
        }
        conn->pcb = NULL;
    }
    return err;
}

static err_t tcpclient_finish_with_status(void *arg, int8_t status) {
    struct tcpclient_connection *conn = (struct tcpclient_connection *)arg;
    conn->status = status;
    return tcpclient_close(arg);
}

static err_t tcp_client_sent(void *arg, struct tcp_pcb *tpcb, u16_t len) {
    struct tcpclient_connection *conn = (struct tcpclient_connection *)arg;
    conn->sent_len += len;

    if (conn->sent_len == conn->data_len) {
        conn->status = STATUS_DATA_SENT;
    }

    return ERR_OK;
}

static err_t tcp_client_connected(void *arg, struct tcp_pcb *tpcb, err_t err) {
    if (err != ERR_OK) {
        return tcpclient_finish_with_status(arg, err);
    }
    struct tcpclient_connection *conn = (struct tcpclient_connection *)arg;
    conn->status = STATUS_CONNECTED;
    return ERR_OK;
}

static err_t tcp_client_poll(void *arg, struct tcp_pcb *tpcb) { return tcpclient_finish_with_status(arg, -1); }

static void tcp_client_err(void *arg, err_t err) {
    if (err != ERR_ABRT) {
        tcpclient_finish_with_status(arg, err);
    }
}

err_t tcp_client_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
    struct tcpclient_connection *conn = (struct tcpclient_connection *)arg;
    const struct pbuf *segment;

    if (NULL == p) {
        return tcpclient_finish_with_status(arg, -1);
    }

    cyw43_arch_lwip_check();
    if (err != ERR_OK) {
        pbuf_free(p);
        return tcpclient_finish_with_status(arg, err);
    }

    if (p->tot_len > 0) {
        tcp_recved(tpcb, p->tot_len);

        for (segment = p; segment != NULL; segment = segment->next) {
            const int8_t *status_bytes = (const int8_t *)segment->payload;
            u16_t index;

            for (index = 0; index < segment->len; ++index) {
                int8_t status = status_bytes[index];

                if (status < 0 || status == STATUS_SUCCESS) {
                    pbuf_free(p);
                    return tcpclient_finish_with_status(arg, status);
                }

                conn->status = status;
            }
        }
    }
    pbuf_free(p);

    return ERR_OK;
}

static void dns_found(const char *hostname, const ip_addr_t *ipaddr, void *arg) {
    struct tcpclient_connection *conn = (struct tcpclient_connection *)arg;
    if (ipaddr != NULL) {
        conn->remote_addr = *ipaddr;
        conn->status = STATUS_DNS_FOUND;
    }
}

static bool tcpclient_wait_for_status(struct tcpclient_connection *conn, int8_t expected_status) {
    while (conn->status != expected_status) {
        if (conn->status < 0) {
            return false;
        }
        sleep_ms(1);
    }

    return true;
}

static bool tcpclient_open(struct tcpclient_connection *conn) {

    cyw43_arch_lwip_begin();
    err_t err = dns_gethostbyname(config.sst_server, &conn->remote_addr, dns_found, conn);
    cyw43_arch_lwip_end();

    if (err == ERR_OK) { // domain name was in cache
        conn->status = STATUS_DNS_FOUND;
    }
    if (!tcpclient_wait_for_status(conn, STATUS_DNS_FOUND)) {
        return false;
    }

    conn->pcb = tcp_new_ip_type(IP_GET_TYPE(&conn->remote_addr));
    if (conn->pcb == NULL) {
        return false;
    }

    tcp_arg(conn->pcb, conn);
    tcp_poll(conn->pcb, tcp_client_poll, POLL_TIME_S * 2);
    tcp_sent(conn->pcb, tcp_client_sent);
    tcp_recv(conn->pcb, tcp_client_recv);
    tcp_err(conn->pcb, tcp_client_err);

    cyw43_arch_lwip_begin();
    err = tcp_connect(conn->pcb, &conn->remote_addr, config.sst_server_port, tcp_client_connected);
    cyw43_arch_lwip_end();

    return err == ERR_OK;
}

static struct tcpclient_connection *tcpclient_create_connection(void) {
    struct tcpclient_connection *conn = malloc(sizeof(struct tcpclient_connection));
    if (conn == NULL) {
        return NULL;
    }

    conn->status = STATUS_INIT;
    conn->sent_len = 0;

    return conn;
}

bool send_file(const char *filename) {
    pico_unique_board_id_t board_id;
    uint8_t header[2 + PICO_UNIQUE_BOARD_ID_SIZE_BYTES + sizeof(FSIZE_t) + (FILENAME_LENGTH - 1)];
    pico_get_unique_board_id(&board_id);

    FILINFO finfo;
    FRESULT fr = f_stat(filename, &finfo);
    if (fr != FR_OK) {
        return false;
    }

    FIL f;
    fr = f_open(&f, filename, FA_OPEN_EXISTING | FA_READ);
    if (!(fr == FR_OK || fr == FR_EXIST)) {
        return false;
    }

    struct tcpclient_connection *conn = tcpclient_create_connection();
    if (conn == NULL) {
        return false;
    }
    if (!tcpclient_open(conn)) {
        tcpclient_finish_with_status(conn, -1);
        return false;
    }

    if (!tcpclient_wait_for_status(conn, STATUS_CONNECTED)) {
        return false;
    }

    conn->data_len = 2 + // the constant string "ID"
                     PICO_UNIQUE_BOARD_ID_SIZE_BYTES + sizeof(FSIZE_t) +
                     (FILENAME_LENGTH - 1) + // we don't send the terminating null byte
                     finfo.fsize;

    memcpy(header, "ID", 2);
    memcpy(header + 2, board_id.id, PICO_UNIQUE_BOARD_ID_SIZE_BYTES);
    memcpy(header + 2 + PICO_UNIQUE_BOARD_ID_SIZE_BYTES, &finfo.fsize, sizeof(FSIZE_t));
    memcpy(header + 2 + PICO_UNIQUE_BOARD_ID_SIZE_BYTES + sizeof(FSIZE_t), filename, FILENAME_LENGTH - 1);

    cyw43_arch_lwip_begin();
    if (tcp_sndbuf(conn->pcb) < sizeof(header) ||
        tcp_write(conn->pcb, header, sizeof(header), TCP_WRITE_FLAG_COPY) != ERR_OK) {
        cyw43_arch_lwip_end();
        tcpclient_finish_with_status(conn, -1);
        return false;
    }
    tcp_output(conn->pcb);
    cyw43_arch_lwip_end();

    if (!tcpclient_wait_for_status(conn, STATUS_HEADER_OK)) {
        return false;
    }

    static uint8_t buffer[READ_BUF_LEN];
    uint br = READ_BUF_LEN;
    FSIZE_t total_read = 0;
    bool needs_retry = false;

    while (true) {
        // Determine how many bytes to send in this turn. This should be the
        // minimum of the available send buffer size and the maximum read
        // length.
        cyw43_arch_lwip_begin();
        u16_t to_read = tcp_sndbuf(conn->pcb);
        cyw43_arch_lwip_end();
        if (to_read > READ_BUF_LEN) {
            to_read = READ_BUF_LEN;
        }

        // Read data from the SST file.
        if (!needs_retry) {
            fr = f_read(&f, buffer, to_read, &br);
            if (fr != FR_OK) {
                tcpclient_finish_with_status(conn, -1);
                return false;
            }
            total_read += br;
        }

        // Write data to TCP stream
        cyw43_arch_lwip_begin();
        err_t err = tcp_write(conn->pcb, buffer, br,
                              TCP_WRITE_FLAG_COPY | (total_read < finfo.fsize ? TCP_WRITE_FLAG_MORE : 0));
        needs_retry = err != ERR_OK;
        tcp_output(conn->pcb);
        cyw43_arch_lwip_end();

        if (total_read == finfo.fsize) {
            break;
        }

        sleep_ms(1);
    }

    f_close(&f);

    if (!tcpclient_wait_for_status(conn, STATUS_SUCCESS)) {
        return false;
    }

    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
    free(conn);

    return true;
}
