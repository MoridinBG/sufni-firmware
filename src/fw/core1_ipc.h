#ifndef CORE1_IPC_H
#define CORE1_IPC_H

#include <stdbool.h>
#include <stdint.h>

#include "hardware/sync.h"

#include "../util/config.h"

#define CORE1_FIFO_FAMILY_MASK           0xFF000000u
#define CORE1_FIFO_ID_MASK               0x0000FFFFu
#define CORE1_FIFO_FAMILY_DISPATCH_CMD   0x01000000u
#define CORE1_FIFO_FAMILY_DISPATCH_EVENT 0x02000000u
#define CORE1_FIFO_FAMILY_STORAGE_CMD    0x03000000u
#define CORE1_FIFO_FAMILY_STORAGE_EVENT  0x04000000u

#define CORE1_FIFO_WORD(family, id) ((uint32_t)((family) | ((uint32_t)(id) & CORE1_FIFO_ID_MASK)))
#define CORE1_FIFO_FAMILY(word)     ((uint32_t)(word) & CORE1_FIFO_FAMILY_MASK)
#define CORE1_FIFO_ID(word)         ((uint32_t)(word) & CORE1_FIFO_ID_MASK)

enum core1_mode {
    CORE1_MODE_IDLE = 0,
    CORE1_MODE_STORAGE,
    CORE1_MODE_TCP_SERVER,
    CORE1_MODE_STOPPING,
    CORE1_MODE_ERROR,
};

enum core1_dispatch_command {
    CORE1_DISPATCH_CMD_ENTER_STORAGE = 1,
    CORE1_DISPATCH_CMD_ENTER_TCP_SERVER = 2,
};

enum core1_dispatch_event {
    CORE1_DISPATCH_EVENT_STORAGE_READY = 1,
    CORE1_DISPATCH_EVENT_TCP_SERVER_READY = 2,
    CORE1_DISPATCH_EVENT_BACKEND_COMPLETE = 3,
    CORE1_DISPATCH_EVENT_BACKEND_ERROR = 4,
};

enum core1_network_session_kind {
    CORE1_NETWORK_SESSION_NONE = 0,
    CORE1_NETWORK_SESSION_SERVE_TCP = 1,
    CORE1_NETWORK_SESSION_SYNC_DATA = 2,
};

enum core1_network_session_request_type {
    CORE1_NETWORK_SESSION_REQUEST_NONE = 0,
    CORE1_NETWORK_SESSION_REQUEST_START = 1,
    CORE1_NETWORK_SESSION_REQUEST_STOP = 2,
};

enum network_session_phase {
    NETWORK_SESSION_PHASE_IDLE = 0,
    NETWORK_SESSION_PHASE_REQUESTED = 1,
    NETWORK_SESSION_PHASE_WIFI_STARTING = 2,
    NETWORK_SESSION_PHASE_NTP_SYNCING = 3,
    NETWORK_SESSION_PHASE_TCP_STARTING = 4,
    NETWORK_SESSION_PHASE_RUNNING = 5,
    NETWORK_SESSION_PHASE_STOPPING = 6,
    NETWORK_SESSION_PHASE_COMPLETED = 7,
    NETWORK_SESSION_PHASE_CANCELLED = 8,
    NETWORK_SESSION_PHASE_ERROR = 9,
};

enum network_session_error {
    NETWORK_SESSION_ERR_NONE = 0,
    NETWORK_SESSION_ERR_BUSY = 1,
    NETWORK_SESSION_ERR_WIFI_START = 2,
    NETWORK_SESSION_ERR_TCP_INIT = 3,
    NETWORK_SESSION_ERR_TCP_RUNTIME = 4,
    NETWORK_SESSION_ERR_CANCELLED = 5,
};

struct core1_network_session_config {
    enum core1_network_session_kind session_kind;
    bool run_ntp;
    bool allow_live_preview;
    bool enable_mdns;
    struct config config_snapshot;
};

struct core1_network_session_request {
    enum core1_network_session_request_type request_type;
    uint32_t request_generation;
    uint32_t session_id;
    struct core1_network_session_config config;
};

struct core1_network_session_stop_request {
    uint32_t request_generation;
    uint32_t session_id;
};

struct core1_network_session_status {
    uint32_t status_generation;
    uint32_t request_generation;
    uint32_t session_id;
    enum network_session_phase phase;
    enum network_session_error error_code;
};

struct core1_network_session_request_mailbox {
    volatile uint32_t publish_generation;
    struct core1_network_session_request request;
};

struct core1_network_session_stop_request_mailbox {
    volatile uint32_t publish_generation;
    struct core1_network_session_stop_request request;
};

struct core1_network_session_status_mailbox {
    volatile uint32_t publish_generation;
    struct core1_network_session_status status;
};

enum storage_session_command {
    STORAGE_CMD_OPEN = 1,
    STORAGE_CMD_DUMP_TRAVEL = 2,
    STORAGE_CMD_DUMP_GPS = 3,
    STORAGE_CMD_DUMP_IMU = 4,
    STORAGE_CMD_FINISH = 5,
    STORAGE_CMD_MARKER = 6,
};

enum storage_session_event {
    STORAGE_EVENT_OPEN_RESULT = 1,
    STORAGE_EVENT_BUFFER_RETURNED = 2,
};

static inline uint32_t core1_mailbox_begin_write(volatile uint32_t *publish_generation) {
    uint32_t write_generation = *publish_generation + 1u;
    *publish_generation = write_generation;
    __dmb();
    return write_generation;
}

static inline void core1_mailbox_end_write(volatile uint32_t *publish_generation, uint32_t write_generation) {
    __dmb();
    *publish_generation = write_generation + 1u;
}

static inline bool core1_mailbox_generation_is_stable(uint32_t publish_generation) {
    return (publish_generation & 1u) == 0u;
}

static inline bool core1_fifo_is_family(uint32_t word, uint32_t family) { return CORE1_FIFO_FAMILY(word) == family; }

#endif // CORE1_IPC_H