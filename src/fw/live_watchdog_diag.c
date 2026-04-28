#include "live_watchdog_diag.h"

#include "../util/log.h"

#include "hardware/watchdog.h"
#include "pico/time.h"

#include <stdbool.h>
#include <string.h>

#define LIVE_WATCHDOG_TIMEOUT_MS    1500u
#define LIVE_WATCHDOG_PEER_GRACE_US 750000u

enum live_watchdog_stale_peer {
    LIVE_WATCHDOG_STALE_NONE = 0,
    LIVE_WATCHDOG_STALE_CORE0,
    LIVE_WATCHDOG_STALE_CORE1,
};

struct live_watchdog_diag_state {
    volatile bool active;
    volatile uint32_t session_id;
    volatile uint32_t core0_last_progress_us;
    volatile uint32_t core1_last_progress_us;
    volatile uint32_t core0_last_marker;
    volatile uint32_t core1_last_marker;
    volatile uint32_t stale_peer;
};

static struct live_watchdog_diag_state live_watchdog_diag;

static uint32_t live_watchdog_diag_elapsed_us(uint32_t now_us, uint32_t then_us) {
    int32_t delta_us = (int32_t)(now_us - then_us);

    if (delta_us <= 0) {
        return 0u;
    }

    return (uint32_t)delta_us;
}

static const char *live_watchdog_diag_core0_marker_name(uint32_t marker) {
    switch (marker) {
        case LIVE_WATCHDOG_CORE0_MARKER_SESSION_START:
            return "start";
        case LIVE_WATCHDOG_CORE0_MARKER_SERVICE:
            return "service";
        case LIVE_WATCHDOG_CORE0_MARKER_TRAVEL_CB:
            return "travel_cb";
        case LIVE_WATCHDOG_CORE0_MARKER_IMU_CB:
            return "imu_cb";
        case LIVE_WATCHDOG_CORE0_MARKER_GPS_TIMER_CB:
            return "gps_timer";
        case LIVE_WATCHDOG_CORE0_MARKER_GPS_FIX:
            return "gps_fix";
        case LIVE_WATCHDOG_CORE0_MARKER_SESSION_STOP:
            return "stop";
        default:
            return "unknown";
    }
}

static const char *live_watchdog_diag_core1_marker_name(uint32_t marker) {
    switch (marker) {
        case LIVE_WATCHDOG_CORE1_MARKER_SERVICE_STARTING:
            return "starting";
        case LIVE_WATCHDOG_CORE1_MARKER_SERVICE_HANDSHAKE:
            return "handshake";
        case LIVE_WATCHDOG_CORE1_MARKER_SERVICE_ACTIVE:
            return "active";
        case LIVE_WATCHDOG_CORE1_MARKER_SERVICE_STOPPING:
            return "stopping";
        case LIVE_WATCHDOG_CORE1_MARKER_SERVICE_STOP_DEFERRED:
            return "stop_deferred";
        case LIVE_WATCHDOG_CORE1_MARKER_WAIT_QUIESCENT:
            return "wait_quiescent";
        case LIVE_WATCHDOG_CORE1_MARKER_LWIP_LOCK_WAIT:
            return "lwip_lock_wait";
        case LIVE_WATCHDOG_CORE1_MARKER_WRITE_FRAME:
            return "write_frame";
        case LIVE_WATCHDOG_CORE1_MARKER_ABORT:
            return "abort";
        default:
            return "unknown";
    }
}

static void live_watchdog_diag_log_state(const char *reason, const char *self_core_name, uint32_t now_us) {
    uint32_t core0_last_progress_us = live_watchdog_diag.core0_last_progress_us;
    uint32_t core1_last_progress_us = live_watchdog_diag.core1_last_progress_us;
    uint32_t core0_last_marker = live_watchdog_diag.core0_last_marker;
    uint32_t core1_last_marker = live_watchdog_diag.core1_last_marker;

    LOG("LIVE", "WDG %s sess=%u self=%s c0=%s(%u) age=%u c1=%s(%u) age=%u\n", reason != NULL ? reason : "snapshot",
        (unsigned)live_watchdog_diag.session_id, self_core_name != NULL ? self_core_name : "n/a",
        live_watchdog_diag_core0_marker_name(core0_last_marker), (unsigned)core0_last_marker,
        (unsigned)live_watchdog_diag_elapsed_us(now_us, core0_last_progress_us),
        live_watchdog_diag_core1_marker_name(core1_last_marker), (unsigned)core1_last_marker,
        (unsigned)live_watchdog_diag_elapsed_us(now_us, core1_last_progress_us));
}

static void live_watchdog_diag_maybe_feed(uint32_t now_us, uint32_t peer_last_progress_us,
                                          enum live_watchdog_stale_peer stale_peer, const char *self_core_name) {
    uint32_t peer_age_us = live_watchdog_diag_elapsed_us(now_us, peer_last_progress_us);

    if (peer_age_us <= LIVE_WATCHDOG_PEER_GRACE_US) {
        if (live_watchdog_diag.stale_peer != LIVE_WATCHDOG_STALE_NONE) {
            live_watchdog_diag_log_state("peer_recovered", self_core_name, now_us);
            live_watchdog_diag.stale_peer = LIVE_WATCHDOG_STALE_NONE;
        }
        watchdog_update();
        return;
    }

    if (live_watchdog_diag.stale_peer != (uint32_t)stale_peer) {
        live_watchdog_diag.stale_peer = (uint32_t)stale_peer;
        live_watchdog_diag_log_state(stale_peer == LIVE_WATCHDOG_STALE_CORE0 ? "core0_stale" : "core1_stale",
                                     self_core_name, now_us);
    }
}

void live_watchdog_diag_init(void) {
    watchdog_disable();
    memset(&live_watchdog_diag, 0, sizeof(live_watchdog_diag));
}

void live_watchdog_diag_session_start(uint32_t session_id) {
    uint32_t now_us = time_us_32();

    live_watchdog_diag = (struct live_watchdog_diag_state){
        .active = true,
        .session_id = session_id,
        .core0_last_progress_us = now_us,
        .core1_last_progress_us = now_us,
        .core0_last_marker = LIVE_WATCHDOG_CORE0_MARKER_SESSION_START,
        .core1_last_marker = LIVE_WATCHDOG_CORE1_MARKER_SERVICE_STARTING,
        .stale_peer = LIVE_WATCHDOG_STALE_NONE,
    };

    LOG("LIVE", "WDG armed sess=%u timeout_ms=%u peer_grace_us=%u\n", (unsigned)session_id,
        (unsigned)LIVE_WATCHDOG_TIMEOUT_MS, (unsigned)LIVE_WATCHDOG_PEER_GRACE_US);
    watchdog_enable(LIVE_WATCHDOG_TIMEOUT_MS, true);
    watchdog_update();
}

void live_watchdog_diag_session_stop(void) {
    if (!live_watchdog_diag.active) {
        return;
    }

    LOG("LIVE", "WDG disarmed sess=%u\n", (unsigned)live_watchdog_diag.session_id);
    live_watchdog_diag.active = false;
    watchdog_disable();
}

void live_watchdog_diag_mark_core0(uint32_t marker) {
    uint32_t now_us;

    if (!live_watchdog_diag.active) {
        return;
    }

    now_us = time_us_32();
    live_watchdog_diag.core0_last_progress_us = now_us;
    live_watchdog_diag.core0_last_marker = marker;
    live_watchdog_diag_maybe_feed(now_us, live_watchdog_diag.core1_last_progress_us, LIVE_WATCHDOG_STALE_CORE1,
                                  "core0");
}

void live_watchdog_diag_mark_core1(uint32_t marker) {
    uint32_t now_us;

    if (!live_watchdog_diag.active) {
        return;
    }

    now_us = time_us_32();
    live_watchdog_diag.core1_last_progress_us = now_us;
    live_watchdog_diag.core1_last_marker = marker;
    live_watchdog_diag_maybe_feed(now_us, live_watchdog_diag.core0_last_progress_us, LIVE_WATCHDOG_STALE_CORE0,
                                  "core1");
}

void live_watchdog_diag_log_snapshot(const char *reason) { live_watchdog_diag_log_state(reason, NULL, time_us_32()); }