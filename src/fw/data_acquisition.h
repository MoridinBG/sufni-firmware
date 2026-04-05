#ifndef DATA_ACQUISITION_H
#define DATA_ACQUISITION_H

#include <stdint.h>

#include "hardware_config.h"
#include "ssd1306.h"
#include "sst.h"

#if HAS_GPS
#include "../sensor/gps/gps_sensor.h"
#endif

extern const uint16_t TRAVEL_SAMPLE_RATE;

#if HAS_IMU
extern const uint16_t IMU_SAMPLE_RATE;
#endif

#if HAS_GPS
extern const uint16_t GPS_SAMPLE_RATE;
void recording_on_gps_fix(const struct gps_telemetry *telemetry);
void recording_start_gps_timer(ssd1306_t *disp);
void recording_stop_gps_timer(void);
#endif

extern struct travel_record travel_databuffer2[];

#if HAS_GPS
extern struct gps_record gps_databuffer2[];
#endif

#if HAS_IMU
extern struct imu_record imu_databuffer2[];
#endif

void recording_reset_buffers(void);
void recording_start(ssd1306_t *disp);
void recording_stop(void);

#endif // DATA_ACQUISITION_H