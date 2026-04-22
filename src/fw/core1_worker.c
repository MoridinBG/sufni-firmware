#include "core1_worker.h"

#include "data_storage.h"

#include "../net/tcpserver.h"
#include "../net/wifi.h"

#include "pico/multicore.h"
#include "pico/platform.h"

#include <stdint.h>

volatile struct core1_worker_status core1_worker_status = {
    .mode = CORE1_MODE_IDLE,
    .last_error = 0,
};

static struct tcpserver core1_tcp_server;
static volatile struct core1_network_session_request_mailbox core1_network_session_request_mailbox;
static volatile struct core1_network_session_stop_request_mailbox core1_network_session_stop_request_mailbox;
static volatile struct core1_network_session_status_mailbox core1_network_session_status_mailbox = {
    .publish_generation = 0,
    .status =
        {
            .status_generation = 0,
            .request_generation = 0,
            .session_id = 0,
            .phase = NETWORK_SESSION_PHASE_IDLE,
            .error_code = NETWORK_SESSION_ERR_NONE,
        },
};
static uint32_t core1_next_network_request_generation = 1u;
static uint32_t core1_next_network_session_id = 1u;
static uint32_t core1_next_network_status_generation = 1u;

struct core1_network_session_stop_context {
    uint32_t request_generation;
    uint32_t session_id;
    uint32_t last_handled_request_generation;
    bool stop_requested;
};

static void core1_send_dispatch_event(enum core1_dispatch_event event_id, int32_t event_data) {
    multicore_fifo_push_blocking(CORE1_FIFO_WORD(CORE1_FIFO_FAMILY_DISPATCH_EVENT, event_id));
    multicore_fifo_push_blocking((uint32_t)event_data);
}

static void core1_set_mode(enum core1_mode mode) { core1_worker_status.mode = mode; }

static void core1_write_network_session_request(const struct core1_network_session_request *request) {
    uint32_t write_generation = core1_mailbox_begin_write(&core1_network_session_request_mailbox.publish_generation);

    core1_network_session_request_mailbox.request = *request;
    core1_mailbox_end_write(&core1_network_session_request_mailbox.publish_generation, write_generation);
}

static void core1_write_network_session_stop_request(const struct core1_network_session_stop_request *request) {
    uint32_t write_generation =
        core1_mailbox_begin_write(&core1_network_session_stop_request_mailbox.publish_generation);

    core1_network_session_stop_request_mailbox.request = *request;
    core1_mailbox_end_write(&core1_network_session_stop_request_mailbox.publish_generation, write_generation);
}

static bool core1_snapshot_network_session_request(struct core1_network_session_request *request) {
    uint32_t before_generation;
    uint32_t after_generation;

    if (request == NULL) {
        return false;
    }

    do {
        before_generation = core1_network_session_request_mailbox.publish_generation;
        if (!core1_mailbox_generation_is_stable(before_generation)) {
            continue;
        }

        __dmb();
        *request = core1_network_session_request_mailbox.request;
        __dmb();

        after_generation = core1_network_session_request_mailbox.publish_generation;
    } while (before_generation != after_generation || !core1_mailbox_generation_is_stable(after_generation));

    return true;
}

static bool core1_snapshot_network_session_stop_request(struct core1_network_session_stop_request *request) {
    uint32_t before_generation;
    uint32_t after_generation;

    if (request == NULL) {
        return false;
    }

    do {
        before_generation = core1_network_session_stop_request_mailbox.publish_generation;
        if (!core1_mailbox_generation_is_stable(before_generation)) {
            continue;
        }

        __dmb();
        *request = core1_network_session_stop_request_mailbox.request;
        __dmb();

        after_generation = core1_network_session_stop_request_mailbox.publish_generation;
    } while (before_generation != after_generation || !core1_mailbox_generation_is_stable(after_generation));

    return true;
}

static void core1_publish_network_session_status(uint32_t request_generation, uint32_t session_id,
                                                 enum network_session_phase phase,
                                                 enum network_session_error error_code) {
    struct core1_network_session_status status = {
        .status_generation = core1_next_network_status_generation++,
        .request_generation = request_generation,
        .session_id = session_id,
        .phase = phase,
        .error_code = error_code,
    };
    uint32_t write_generation = core1_mailbox_begin_write(&core1_network_session_status_mailbox.publish_generation);

    core1_network_session_status_mailbox.status = status;
    core1_mailbox_end_write(&core1_network_session_status_mailbox.publish_generation, write_generation);
}

static bool core1_network_session_should_stop(void *context) {
    struct core1_network_session_stop_context *stop_context = context;
    struct core1_network_session_stop_request request;

    if (stop_context == NULL) {
        return false;
    }

    if (stop_context->stop_requested) {
        return true;
    }

    if (!core1_snapshot_network_session_stop_request(&request) || request.session_id != stop_context->session_id ||
        request.request_generation <= stop_context->last_handled_request_generation) {
        return false;
    }

    stop_context->last_handled_request_generation = request.request_generation;
    stop_context->stop_requested = true;
    core1_set_mode(CORE1_MODE_STOPPING);
    core1_publish_network_session_status(stop_context->request_generation, stop_context->session_id,
                                         NETWORK_SESSION_PHASE_STOPPING, NETWORK_SESSION_ERR_NONE);
    return true;
}

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

bool core1_read_network_session_status(struct core1_network_session_status *status) {
    uint32_t before_generation;
    uint32_t after_generation;

    if (status == NULL) {
        return false;
    }

    do {
        before_generation = core1_network_session_status_mailbox.publish_generation;
        if (!core1_mailbox_generation_is_stable(before_generation)) {
            continue;
        }

        __dmb();
        *status = core1_network_session_status_mailbox.status;
        __dmb();

        after_generation = core1_network_session_status_mailbox.publish_generation;
    } while (before_generation != after_generation || !core1_mailbox_generation_is_stable(after_generation));

    return true;
}

static void core1_run_storage_backend(void) {
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
    struct core1_network_session_request request;
    struct core1_network_session_stop_context stop_context;
    struct tcpserver_options options;
    uint32_t session_id;
    bool wifi_started = false;

    core1_set_mode(CORE1_MODE_TCP_SERVER);

    if (!core1_snapshot_network_session_request(&request) ||
        request.request_type != CORE1_NETWORK_SESSION_REQUEST_START) {
        status = PICO_ERROR_GENERIC;
        core1_worker_status.last_error = status;
        core1_set_mode(CORE1_MODE_ERROR);
        core1_set_mode(CORE1_MODE_IDLE);
        return;
    }

    session_id = core1_next_network_session_id++;
    options = (struct tcpserver_options){
        .allow_live_preview = request.config.allow_live_preview,
        .enable_mdns = request.config.enable_mdns,
    };

    stop_context = (struct core1_network_session_stop_context){
        .request_generation = request.request_generation,
        .session_id = session_id,
        .last_handled_request_generation = request.request_generation,
        .stop_requested = false,
    };

    core1_publish_network_session_status(request.request_generation, session_id, NETWORK_SESSION_PHASE_REQUESTED,
                                         NETWORK_SESSION_ERR_NONE);
    core1_publish_network_session_status(request.request_generation, session_id, NETWORK_SESSION_PHASE_WIFI_STARTING,
                                         NETWORK_SESSION_ERR_NONE);

    if (!wifi_start_with_config(&request.config.config_snapshot, request.config.run_ntp)) {
        status = PICO_ERROR_GENERIC;
        core1_worker_status.last_error = status;
        core1_set_mode(CORE1_MODE_ERROR);
        core1_publish_network_session_status(request.request_generation, session_id, NETWORK_SESSION_PHASE_ERROR,
                                             NETWORK_SESSION_ERR_WIFI_START);
        core1_set_mode(CORE1_MODE_IDLE);
        return;
    }
    wifi_started = true;

    if (core1_network_session_should_stop(&stop_context)) {
        wifi_stop();
        core1_publish_network_session_status(request.request_generation, session_id, NETWORK_SESSION_PHASE_CANCELLED,
                                             NETWORK_SESSION_ERR_CANCELLED);
        core1_set_mode(CORE1_MODE_IDLE);
        return;
    }

    core1_publish_network_session_status(request.request_generation, session_id, NETWORK_SESSION_PHASE_TCP_STARTING,
                                         NETWORK_SESSION_ERR_NONE);

    if (!tcpserver_init(&core1_tcp_server, &options)) {
        status = PICO_ERROR_GENERIC;
        core1_worker_status.last_error = status;
        core1_set_mode(CORE1_MODE_ERROR);
        if (wifi_started) {
            wifi_stop();
        }
        core1_publish_network_session_status(request.request_generation, session_id, NETWORK_SESSION_PHASE_ERROR,
                                             NETWORK_SESSION_ERR_TCP_INIT);
        core1_set_mode(CORE1_MODE_IDLE);
        return;
    }

    if (core1_network_session_should_stop(&stop_context)) {
        if (!tcpserver_run(&core1_tcp_server, core1_network_session_should_stop, &stop_context)) {
            status = PICO_ERROR_GENERIC;
            core1_worker_status.last_error = status;
            core1_set_mode(CORE1_MODE_ERROR);
            if (wifi_started) {
                wifi_stop();
            }
            core1_publish_network_session_status(request.request_generation, session_id, NETWORK_SESSION_PHASE_ERROR,
                                                 NETWORK_SESSION_ERR_TCP_RUNTIME);
        } else {
            if (wifi_started) {
                wifi_stop();
            }
            core1_publish_network_session_status(request.request_generation, session_id,
                                                 NETWORK_SESSION_PHASE_CANCELLED, NETWORK_SESSION_ERR_CANCELLED);
        }

        core1_set_mode(CORE1_MODE_IDLE);
        return;
    }

    core1_publish_network_session_status(request.request_generation, session_id, NETWORK_SESSION_PHASE_RUNNING,
                                         NETWORK_SESSION_ERR_NONE);

    if (!tcpserver_run(&core1_tcp_server, core1_network_session_should_stop, &stop_context)) {
        status = PICO_ERROR_GENERIC;
        core1_worker_status.last_error = status;
        core1_set_mode(CORE1_MODE_ERROR);
        core1_publish_network_session_status(request.request_generation, session_id, NETWORK_SESSION_PHASE_ERROR,
                                             NETWORK_SESSION_ERR_TCP_RUNTIME);
    } else {
        core1_publish_network_session_status(request.request_generation, session_id, NETWORK_SESSION_PHASE_COMPLETED,
                                             NETWORK_SESSION_ERR_NONE);
    }

    if (wifi_started) {
        wifi_stop();
    }

    core1_set_mode(CORE1_MODE_IDLE);
}

enum core1_mode core1_get_mode(void) { return core1_worker_status.mode; }

bool core1_request_network_session_start(const struct core1_network_session_config *config,
                                         uint32_t *request_generation) {
    struct core1_network_session_request request;

    if (config == NULL || core1_get_mode() != CORE1_MODE_IDLE) {
        return false;
    }

    request = (struct core1_network_session_request){
        .request_type = CORE1_NETWORK_SESSION_REQUEST_START,
        .request_generation = core1_next_network_request_generation++,
        .session_id = 0,
        .config = *config,
    };

    core1_write_network_session_request(&request);
    if (request_generation != NULL) {
        *request_generation = request.request_generation;
    }

    multicore_fifo_push_blocking(CORE1_FIFO_WORD(CORE1_FIFO_FAMILY_DISPATCH_CMD, CORE1_DISPATCH_CMD_ENTER_TCP_SERVER));
    return true;
}

bool core1_request_network_session_stop(uint32_t session_id, uint32_t *request_generation) {
    struct core1_network_session_stop_request request = {0};

    if (session_id == 0 || core1_get_mode() == CORE1_MODE_IDLE) {
        return false;
    }

    request.request_generation = core1_next_network_request_generation++;
    request.session_id = session_id;

    core1_write_network_session_stop_request(&request);
    if (request_generation != NULL) {
        *request_generation = request.request_generation;
    }

    return true;
}

bool core1_request_mode(enum core1_mode mode) {
    enum core1_dispatch_command command;

    if (core1_get_mode() != CORE1_MODE_IDLE) {
        return false;
    }

    switch (mode) {
        case CORE1_MODE_STORAGE:
            command = CORE1_DISPATCH_CMD_ENTER_STORAGE;
            break;
        default:
            return false;
    }

    multicore_fifo_push_blocking(CORE1_FIFO_WORD(CORE1_FIFO_FAMILY_DISPATCH_CMD, command));
    return true;
}

bool core1_wait_for_event(enum core1_dispatch_event expected_event, int32_t *event_data) {
    enum core1_dispatch_event event_id;

    if (!core1_read_event(&event_id, event_data)) {
        return false;
    }

    return event_id == expected_event;
}

void core1_worker_main(void) {
    core1_set_mode(CORE1_MODE_IDLE);
    core1_publish_network_session_status(0, 0, NETWORK_SESSION_PHASE_IDLE, NETWORK_SESSION_ERR_NONE);

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