#ifndef LIVE_WATCHDOG_DIAG_H
#define LIVE_WATCHDOG_DIAG_H

#include <stdint.h>

enum live_watchdog_core0_marker {
    LIVE_WATCHDOG_CORE0_MARKER_SESSION_START = 1,
    LIVE_WATCHDOG_CORE0_MARKER_SERVICE,
    LIVE_WATCHDOG_CORE0_MARKER_TRAVEL_CB,
    LIVE_WATCHDOG_CORE0_MARKER_IMU_CB,
    LIVE_WATCHDOG_CORE0_MARKER_GPS_TIMER_CB,
    LIVE_WATCHDOG_CORE0_MARKER_GPS_FIX,
    LIVE_WATCHDOG_CORE0_MARKER_SESSION_STOP,
};

enum live_watchdog_core1_marker {
    LIVE_WATCHDOG_CORE1_MARKER_SERVICE_STARTING = 1,
    LIVE_WATCHDOG_CORE1_MARKER_SERVICE_HANDSHAKE,
    LIVE_WATCHDOG_CORE1_MARKER_SERVICE_ACTIVE,
    LIVE_WATCHDOG_CORE1_MARKER_SERVICE_STOPPING,
    LIVE_WATCHDOG_CORE1_MARKER_SERVICE_STOP_DEFERRED,
    LIVE_WATCHDOG_CORE1_MARKER_WAIT_QUIESCENT,
    LIVE_WATCHDOG_CORE1_MARKER_LWIP_LOCK_WAIT,
    LIVE_WATCHDOG_CORE1_MARKER_WRITE_FRAME,
    LIVE_WATCHDOG_CORE1_MARKER_ABORT,
};

void live_watchdog_diag_init(void);
void live_watchdog_diag_session_start(uint32_t session_id);
void live_watchdog_diag_session_stop(void);
void live_watchdog_diag_mark_core0(uint32_t marker);
void live_watchdog_diag_mark_core1(uint32_t marker);
void live_watchdog_diag_log_snapshot(const char *reason);

#endif // LIVE_WATCHDOG_DIAG_H