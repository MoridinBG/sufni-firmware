#include "sensor_setup.h"

#if HAS_GPS
#include "../sensor/gps/lc76g.h"

#if GPS_MODULE == GPS_LC76G
struct gps_sensor gps = {
    .type = GPS_TYPE_LC76G,
    .protocol = GPS_PROTOCOL_UART,
    .comm.uart = {GPS_UART_INST, GPS_PIN_TX, GPS_PIN_RX, GPS_BAUD_RATE},
    .available = false,
    .on_fix = NULL,
    .init = lc76g_init,
    .configure = lc76g_configure,
    .process = lc76g_process,
    .send_command = lc76g_send_command,
    .hot_start = lc76g_hot_start,
    .cold_start = lc76g_cold_start,
    .power_on = lc76g_power_on,
    .power_off = lc76g_power_off,
};
#else
struct gps_sensor gps = {.available = false};
#endif
#endif

#if HAS_IMU
#include "../sensor/imu/lsm6dso.h"
#include "../sensor/imu/mpu6050.h"

#if IMU_FRAME == IMU_MPU6050
struct imu_sensor imu_frame = {
    .type = IMU_TYPE_MPU6050,
    .protocol = IMU_PROTOCOL_I2C,
    .comm.i2c = {IMU_FRAME_I2C_INST, IMU_FRAME_ADDRESS, IMU_FRAME_PIN_SDA, IMU_FRAME_PIN_SCL},
    .available = false,
    .calibration = IMU_CALIBRATION_DEFAULT,
    .gyro_temp_coeff = MPU6050_GYRO_TEMP_COEFF,
    .accel_temp_coeff = MPU6050_ACCEL_TEMP_COEFF,
    .temp_scale = MPU6050_TEMP_SCALE,
    .temp_offset = MPU6050_TEMP_OFFSET,
    .init = mpu6050_init,
    .check_availability = mpu6050_check_availability,
    .read_raw = mpu6050_read_raw,
    .read_temperature = mpu6050_read_temperature,
    .temperature_celsius = mpu6050_temperature_celsius,
};
#elif IMU_FRAME == IMU_LSM6DSO
struct imu_sensor imu_frame = {
    .type = IMU_TYPE_LSM6DSO,
#ifdef IMU_FRAME_SPI
    .protocol = IMU_PROTOCOL_SPI,
    .comm.spi = {IMU_FRAME_SPI_INST, IMU_FRAME_PIN_CS, IMU_FRAME_PIN_SCK, IMU_FRAME_PIN_MOSI, IMU_FRAME_PIN_MISO},
#else
    .protocol = IMU_PROTOCOL_I2C,
    .comm.i2c = {IMU_FRAME_I2C_INST, IMU_FRAME_ADDRESS, IMU_FRAME_PIN_SDA, IMU_FRAME_PIN_SCL},
#endif
    .available = false,
    .calibration = IMU_CALIBRATION_DEFAULT,
    .gyro_temp_coeff = LSM6DSO_GYRO_TEMP_COEFF,
    .accel_temp_coeff = LSM6DSO_ACCEL_TEMP_COEFF,
    .temp_scale = LSM6DSO_TEMP_SCALE,
    .temp_offset = LSM6DSO_TEMP_OFFSET,
    .init = lsm6dso_init,
    .check_availability = lsm6dso_check_availability,
    .read_raw = lsm6dso_read_raw,
    .read_temperature = lsm6dso_read_temperature,
    .temperature_celsius = lsm6dso_temperature_celsius,
};
#else
struct imu_sensor imu_frame = {.available = false};
#endif

#if IMU_FORK == IMU_MPU6050
struct imu_sensor imu_fork = {
    .type = IMU_TYPE_MPU6050,
    .protocol = IMU_PROTOCOL_I2C,
    .comm.i2c = {IMU_FORK_I2C_INST, IMU_FORK_ADDRESS, IMU_FORK_PIN_SDA, IMU_FORK_PIN_SCL},
    .available = false,
    .calibration = IMU_CALIBRATION_DEFAULT,
    .gyro_temp_coeff = MPU6050_GYRO_TEMP_COEFF,
    .accel_temp_coeff = MPU6050_ACCEL_TEMP_COEFF,
    .temp_scale = MPU6050_TEMP_SCALE,
    .temp_offset = MPU6050_TEMP_OFFSET,
    .init = mpu6050_init,
    .check_availability = mpu6050_check_availability,
    .read_raw = mpu6050_read_raw,
    .read_temperature = mpu6050_read_temperature,
    .temperature_celsius = mpu6050_temperature_celsius,
};
#elif IMU_FORK == IMU_LSM6DSO
struct imu_sensor imu_fork = {
    .type = IMU_TYPE_LSM6DSO,
#ifdef IMU_FORK_SPI
    .protocol = IMU_PROTOCOL_SPI,
    .comm.spi = {IMU_FORK_SPI_INST, IMU_FORK_PIN_CS, IMU_FORK_PIN_SCK, IMU_FORK_PIN_MOSI, IMU_FORK_PIN_MISO},
#else
    .protocol = IMU_PROTOCOL_I2C,
    .comm.i2c = {IMU_FORK_I2C_INST, IMU_FORK_ADDRESS, IMU_FORK_PIN_SDA, IMU_FORK_PIN_SCL},
#endif
    .available = false,
    .calibration = IMU_CALIBRATION_DEFAULT,
    .gyro_temp_coeff = LSM6DSO_GYRO_TEMP_COEFF,
    .accel_temp_coeff = LSM6DSO_ACCEL_TEMP_COEFF,
    .temp_scale = LSM6DSO_TEMP_SCALE,
    .temp_offset = LSM6DSO_TEMP_OFFSET,
    .init = lsm6dso_init,
    .check_availability = lsm6dso_check_availability,
    .read_raw = lsm6dso_read_raw,
    .read_temperature = lsm6dso_read_temperature,
    .temperature_celsius = lsm6dso_temperature_celsius,
};
#else
struct imu_sensor imu_fork = {.available = false};
#endif

#if IMU_REAR == IMU_MPU6050
struct imu_sensor imu_rear = {
    .type = IMU_TYPE_MPU6050,
    .protocol = IMU_PROTOCOL_I2C,
    .comm.i2c = {IMU_REAR_I2C_INST, IMU_REAR_ADDRESS, IMU_REAR_PIN_SDA, IMU_REAR_PIN_SCL},
    .available = false,
    .calibration = IMU_CALIBRATION_DEFAULT,
    .gyro_temp_coeff = MPU6050_GYRO_TEMP_COEFF,
    .accel_temp_coeff = MPU6050_ACCEL_TEMP_COEFF,
    .temp_scale = MPU6050_TEMP_SCALE,
    .temp_offset = MPU6050_TEMP_OFFSET,
    .init = mpu6050_init,
    .check_availability = mpu6050_check_availability,
    .read_raw = mpu6050_read_raw,
    .read_temperature = mpu6050_read_temperature,
    .temperature_celsius = mpu6050_temperature_celsius,
};
#elif IMU_REAR == IMU_LSM6DSO
struct imu_sensor imu_rear = {
    .type = IMU_TYPE_LSM6DSO,
#ifdef IMU_REAR_SPI
    .protocol = IMU_PROTOCOL_SPI,
    .comm.spi = {IMU_REAR_SPI_INST, IMU_REAR_PIN_CS, IMU_REAR_PIN_SCK, IMU_REAR_PIN_MOSI, IMU_REAR_PIN_MISO},
#else
    .protocol = IMU_PROTOCOL_I2C,
    .comm.i2c = {IMU_REAR_I2C_INST, IMU_REAR_ADDRESS, IMU_REAR_PIN_SDA, IMU_REAR_PIN_SCL},
#endif
    .available = false,
    .calibration = IMU_CALIBRATION_DEFAULT,
    .gyro_temp_coeff = LSM6DSO_GYRO_TEMP_COEFF,
    .accel_temp_coeff = LSM6DSO_ACCEL_TEMP_COEFF,
    .temp_scale = LSM6DSO_TEMP_SCALE,
    .temp_offset = LSM6DSO_TEMP_OFFSET,
    .init = lsm6dso_init,
    .check_availability = lsm6dso_check_availability,
    .read_raw = lsm6dso_read_raw,
    .read_temperature = lsm6dso_read_temperature,
    .temperature_celsius = lsm6dso_temperature_celsius,
};
#else
struct imu_sensor imu_rear = {.available = false};
#endif
#endif