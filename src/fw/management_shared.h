#ifndef MANAGEMENT_SHARED_H
#define MANAGEMENT_SHARED_H

#include <stdint.h>

#include "hardware/sync.h"
#include "pico.h"

#include "../util/config.h"

enum management_core_command {
    MGMT_CORE_CMD_NONE = 0,
    MGMT_CORE_CMD_APPLY_CONFIG = 1,
    MGMT_CORE_CMD_SET_TIME = 2,
};

enum management_core_state {
    MGMT_CORE_STATE_IDLE = 0,
    MGMT_CORE_STATE_REQUEST_READY = 1,
    MGMT_CORE_STATE_RESPONSE_READY = 2,
};

struct management_core_shared {
    volatile enum management_core_state state;
    uint32_t request_id;
    enum management_core_command command;
    union {
        struct {
            struct config snapshot;
        } apply_config;
        struct {
            uint32_t utc_seconds;
            uint32_t micros;
        } set_time;
    } request;
    int32_t result_code;
};

extern struct management_core_shared management_core_shared;

void management_shared_reset(void);

static inline enum management_core_state management_shared_get_state(void) {
    enum management_core_state state = management_core_shared.state;
    __dmb();
    return state;
}

static inline void management_shared_publish_request(void) {
    __dmb();
    management_core_shared.state = MGMT_CORE_STATE_REQUEST_READY;
}

static inline void management_shared_publish_response(int32_t result_code) {
    management_core_shared.result_code = result_code;
    __dmb();
    management_core_shared.state = MGMT_CORE_STATE_RESPONSE_READY;
}

static inline void management_shared_complete_response(void) {
    __dmb();
    management_core_shared.state = MGMT_CORE_STATE_IDLE;
}

#endif // MANAGEMENT_SHARED_H