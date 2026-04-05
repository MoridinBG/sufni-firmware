#ifndef SENSOR_SETUP_H
#define SENSOR_SETUP_H

#include "hardware_config.h"

#if HAS_GPS
#include "../sensor/gps/gps_sensor.h"

extern struct gps_sensor gps;
#endif

#if HAS_IMU
#include "../sensor/imu/imu_sensor.h"

extern struct imu_sensor imu_frame;
extern struct imu_sensor imu_fork;
extern struct imu_sensor imu_rear;
#endif

#endif // SENSOR_SETUP_H