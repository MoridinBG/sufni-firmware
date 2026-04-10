#ifndef LIVE_STREAM_CORE0_H
#define LIVE_STREAM_CORE0_H

#include <stdbool.h>

#include "live_stream_shared.h"

#if HAS_GPS
#include "../sensor/gps/gps_sensor.h"
#endif

bool live_stream_core0_start(const struct live_start_request *req, struct live_start_response *resp);
void live_stream_core0_stop(void);
bool live_stream_core0_active(void);
void live_stream_core0_service(void);

#if HAS_GPS
void live_stream_core0_on_gps_fix(const struct gps_telemetry *telemetry);
void gps_fix_router_on_fix(const struct gps_telemetry *telemetry);
#endif

#endif // LIVE_STREAM_CORE0_H