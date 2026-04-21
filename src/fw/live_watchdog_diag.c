#include "live_watchdog_diag.h"

#include "hardware/watchdog.h"
#include "pico/time.h"

#include <stdbool.h>
#include <string.h>

#define LIVE_WATCHDOG_TIMEOUT_MS    1500u
#define LIVE_WATCHDOG_PEER_GRACE_US 750000u

struct live_watchdog_diag_state {
    volatile bool active;
    volatile uint32_t core0_last_progress_us;
    volatile uint32_t core1_last_progress_us;
};

static struct live_watchdog_diag_state live_watchdog_diag;

static void live_watchdog_diag_maybe_feed(uint32_t now_us, uint32_t peer_last_progress_us) {
    if ((uint32_t)(now_us - peer_last_progress_us) <= LIVE_WATCHDOG_PEER_GRACE_US) {
        watchdog_update();
    }
}

void live_watchdog_diag_init(void) {
    watchdog_disable();
    memset(&live_watchdog_diag, 0, sizeof(live_watchdog_diag));
}

void live_watchdog_diag_session_start(uint32_t session_id) {
    uint32_t now_us = time_us_32();

    (void)session_id;

    live_watchdog_diag = (struct live_watchdog_diag_state){
        .active = true,
        .core0_last_progress_us = now_us,
        .core1_last_progress_us = now_us,
    };

    watchdog_enable(LIVE_WATCHDOG_TIMEOUT_MS, true);
    watchdog_update();
}

void live_watchdog_diag_session_stop(void) {
    if (!live_watchdog_diag.active) {
        return;
    }

    live_watchdog_diag.active = false;
    watchdog_disable();
}

void live_watchdog_diag_mark_core0(uint32_t marker) {
    uint32_t now_us;

    (void)marker;

    if (!live_watchdog_diag.active) {
        return;
    }

    now_us = time_us_32();
    live_watchdog_diag.core0_last_progress_us = now_us;
    live_watchdog_diag_maybe_feed(now_us, live_watchdog_diag.core1_last_progress_us);
}

void live_watchdog_diag_mark_core1(uint32_t marker) {
    uint32_t now_us;

    (void)marker;

    if (!live_watchdog_diag.active) {
        return;
    }

    now_us = time_us_32();
    live_watchdog_diag.core1_last_progress_us = now_us;
    live_watchdog_diag_maybe_feed(now_us, live_watchdog_diag.core0_last_progress_us);
}