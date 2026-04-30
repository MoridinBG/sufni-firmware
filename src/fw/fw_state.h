#ifndef FW_STATE_H
#define FW_STATE_H

#include <stdbool.h>
#include <stdint.h>

#include "hardware_config.h"

enum state {
    IDLE,
    SLEEP,
    WAKING,
    REC_START,
    GPS_WAIT,
    RECORD,
    REC_STOP,
    SERVE_TCP,
    MSC,
};

#define STATES_COUNT 9

extern volatile enum state state;
extern volatile bool marker_pending;

#if HAS_GPS
extern volatile bool skip_gps_recording;
extern volatile uint8_t gps_last_satellites;
extern volatile float gps_last_epe;
#endif

#endif // FW_STATE_H