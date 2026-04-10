#include "core1_worker.h"

#include "data_storage.h"

#include "../net/tcpserver.h"

#include "pico/multicore.h"
#include "pico/platform.h"

#include <stdint.h>

volatile struct core1_worker_status core1_worker_status = {
    .mode = CORE1_MODE_IDLE,
    .last_error = 0,
};

static struct tcpserver core1_tcp_server;
static volatile bool core1_stop_requested = false;

static void core1_send_dispatch_event(enum core1_dispatch_event event_id, int32_t event_data) {
    multicore_fifo_push_blocking(CORE1_FIFO_WORD(CORE1_FIFO_FAMILY_DISPATCH_EVENT, event_id));
    multicore_fifo_push_blocking((uint32_t)event_data);
}

static void core1_set_mode(enum core1_mode mode) { core1_worker_status.mode = mode; }

static bool core1_read_event(enum core1_dispatch_event *event_id, int32_t *event_data) {
    uint32_t event_word = multicore_fifo_pop_blocking();
    uint32_t event_payload = multicore_fifo_pop_blocking();

    if (!core1_fifo_is_family(event_word, CORE1_FIFO_FAMILY_DISPATCH_EVENT)) {
        return false;
    }

    if (event_id != NULL) {
        *event_id = (enum core1_dispatch_event)CORE1_FIFO_ID(event_word);
    }

    if (event_data != NULL) {
        *event_data = (int32_t)event_payload;
    }

    return true;
}

static void core1_run_storage_backend(void) {
    core1_stop_requested = false;
    core1_set_mode(CORE1_MODE_STORAGE);
    core1_send_dispatch_event(CORE1_DISPATCH_EVENT_STORAGE_READY, 0);

    int status = storage_session_run();
    if (status < 0) {
        core1_worker_status.last_error = status;
        core1_set_mode(CORE1_MODE_ERROR);
    }

    core1_set_mode(CORE1_MODE_IDLE);
    core1_send_dispatch_event(CORE1_DISPATCH_EVENT_BACKEND_COMPLETE, status);
}

static void core1_run_tcp_backend(void) {
    int status = 0;

    core1_stop_requested = false;
    core1_set_mode(CORE1_MODE_TCP_SERVER);

    if (!tcpserver_init(&core1_tcp_server)) {
        core1_worker_status.last_error = PICO_ERROR_GENERIC;
        core1_set_mode(CORE1_MODE_ERROR);
        core1_send_dispatch_event(CORE1_DISPATCH_EVENT_BACKEND_ERROR, PICO_ERROR_GENERIC);
        core1_set_mode(CORE1_MODE_IDLE);
        return;
    }

    core1_send_dispatch_event(CORE1_DISPATCH_EVENT_TCP_SERVER_READY, 0);

    if (!tcpserver_run(&core1_tcp_server, &core1_stop_requested)) {
        status = PICO_ERROR_GENERIC;
        core1_worker_status.last_error = status;
        core1_set_mode(CORE1_MODE_ERROR);
    }

    core1_set_mode(CORE1_MODE_IDLE);
    core1_send_dispatch_event(CORE1_DISPATCH_EVENT_BACKEND_COMPLETE, status);
}

enum core1_mode core1_get_mode(void) { return core1_worker_status.mode; }

bool core1_request_mode(enum core1_mode mode) {
    enum core1_dispatch_command command;

    if (core1_get_mode() != CORE1_MODE_IDLE) {
        return false;
    }

    switch (mode) {
        case CORE1_MODE_STORAGE:
            command = CORE1_DISPATCH_CMD_ENTER_STORAGE;
            break;
        case CORE1_MODE_TCP_SERVER:
            command = CORE1_DISPATCH_CMD_ENTER_TCP_SERVER;
            break;
        default:
            return false;
    }

    multicore_fifo_push_blocking(CORE1_FIFO_WORD(CORE1_FIFO_FAMILY_DISPATCH_CMD, command));
    return true;
}

bool core1_request_stop(void) {
    if (core1_get_mode() == CORE1_MODE_IDLE) {
        return false;
    }

    core1_stop_requested = true;
    return true;
}

bool core1_wait_next_event(enum core1_dispatch_event *event_id, int32_t *event_data) {
    return core1_read_event(event_id, event_data);
}

bool core1_wait_for_event(enum core1_dispatch_event expected_event, int32_t *event_data) {
    enum core1_dispatch_event event_id;

    if (!core1_read_event(&event_id, event_data)) {
        return false;
    }

    return event_id == expected_event;
}

bool core1_poll_event(enum core1_dispatch_event *event_id, int32_t *event_data) {
    if (!multicore_fifo_rvalid()) {
        return false;
    }

    return core1_read_event(event_id, event_data);
}

void core1_worker_main(void) {
    core1_set_mode(CORE1_MODE_IDLE);

    while (true) {
        uint32_t command_word = multicore_fifo_pop_blocking();
        if (!core1_fifo_is_family(command_word, CORE1_FIFO_FAMILY_DISPATCH_CMD)) {
            core1_worker_status.last_error = PICO_ERROR_GENERIC;
            continue;
        }

        switch (CORE1_FIFO_ID(command_word)) {
            case CORE1_DISPATCH_CMD_ENTER_STORAGE:
                core1_run_storage_backend();
                break;
            case CORE1_DISPATCH_CMD_ENTER_TCP_SERVER:
                core1_run_tcp_backend();
                break;
            default:
                core1_worker_status.last_error = PICO_ERROR_GENERIC;
                core1_set_mode(CORE1_MODE_ERROR);
                core1_send_dispatch_event(CORE1_DISPATCH_EVENT_BACKEND_ERROR, PICO_ERROR_GENERIC);
                core1_set_mode(CORE1_MODE_IDLE);
                break;
        }
    }
}