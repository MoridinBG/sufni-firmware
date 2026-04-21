#ifndef LIVE_WATCHDOG_DIAG_H
#define LIVE_WATCHDOG_DIAG_H

#include <stdint.h>

void live_watchdog_diag_init(void);
void live_watchdog_diag_session_start(uint32_t session_id);
void live_watchdog_diag_session_stop(void);
void live_watchdog_diag_mark_core0(uint32_t marker);
void live_watchdog_diag_mark_core1(uint32_t marker);

#endif // LIVE_WATCHDOG_DIAG_H