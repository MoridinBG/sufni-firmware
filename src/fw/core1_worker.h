#ifndef CORE1_WORKER_H
#define CORE1_WORKER_H

#include <stdbool.h>
#include <stdint.h>

#include "core1_ipc.h"

struct core1_worker_status {
    volatile enum core1_mode mode;
    volatile int32_t last_error;
};

extern volatile struct core1_worker_status core1_worker_status;

void core1_worker_main(void);
bool core1_request_mode(enum core1_mode mode);
bool core1_request_stop(void);
bool core1_wait_next_event(enum core1_dispatch_event *event_id, int32_t *event_data);
bool core1_wait_for_event(enum core1_dispatch_event expected_event, int32_t *event_data);
bool core1_poll_event(enum core1_dispatch_event *event_id, int32_t *event_data);
enum core1_mode core1_get_mode(void);

#endif // CORE1_WORKER_H