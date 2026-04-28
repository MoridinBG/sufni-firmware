#include "mpu6050.h"
#include "../../util/i2c_safe.h"
#include "../../util/log.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "imu_sensor.h"
#include "pico/error.h"
#include "pico/time.h"
#include <stdio.h>
#include <string.h>

#define MPU6050_ADDR 0x68

#define REG_GYRO_CONFIG  0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_XOUT_H 0x3B
#define REG_TEMP_OUT_H   0x41
#define REG_GYRO_XOUT_H  0x43
#define REG_PWR_MGMT_1   0x6B
#define REG_WHO_AM_I     0x75

#define SENSOR_I2C_BAUDRATE      1000000u
#define I2C_RECOVERY_PULSE_COUNT 9u
#define I2C_RECOVERY_DELAY_US    5u
#define MPU6050_I2C_TIMEOUT_US   2000u

static bool mpu6050_recover_bus(struct imu_sensor *imu) {
    return i2c_recover_bus(imu->comm.i2c.instance, imu->comm.i2c.sda_gpio, imu->comm.i2c.scl_gpio, SENSOR_I2C_BAUDRATE,
                           I2C_RECOVERY_PULSE_COUNT, I2C_RECOVERY_DELAY_US);
}

static void mpu6050_maybe_recover_timeout(struct imu_sensor *imu, int rc) {
    if (rc != PICO_ERROR_TIMEOUT) {
        return;
    }

    mpu6050_recover_bus(imu);
}

static void write_register(struct imu_sensor *imu, uint8_t reg, uint8_t data) {
    if (imu->protocol == IMU_PROTOCOL_I2C) {
        uint8_t buf[2] = {reg, data};
        int rc = i2c_write_timeout_bounded_us(imu->comm.i2c.instance, imu->comm.i2c.address, buf, 2, false,
                                              MPU6050_I2C_TIMEOUT_US);
        if (rc < 0) {
            mpu6050_maybe_recover_timeout(imu, rc);
        }
    }
}

static uint8_t read_register(struct imu_sensor *imu, uint8_t reg) {
    uint8_t buf[1] = {0};
    if (imu->protocol == IMU_PROTOCOL_I2C) {
        int rc = i2c_write_timeout_bounded_us(imu->comm.i2c.instance, imu->comm.i2c.address, &reg, 1, true,
                                              MPU6050_I2C_TIMEOUT_US);
        if (rc < 0) {
            mpu6050_maybe_recover_timeout(imu, rc);
            return 0;
        }

        rc = i2c_read_timeout_bounded_us(imu->comm.i2c.instance, imu->comm.i2c.address, buf, 1, false,
                                         MPU6050_I2C_TIMEOUT_US);
        if (rc < 0) {
            mpu6050_maybe_recover_timeout(imu, rc);
            return 0;
        }
    }
    return buf[0];
}

static bool read_registers(struct imu_sensor *imu, uint8_t reg, uint8_t *buf, size_t len) {
    if (imu->protocol == IMU_PROTOCOL_I2C) {
        int rc = i2c_write_timeout_bounded_us(imu->comm.i2c.instance, imu->comm.i2c.address, &reg, 1, true,
                                              MPU6050_I2C_TIMEOUT_US);
        if (rc < 0) {
            mpu6050_maybe_recover_timeout(imu, rc);
            return false;
        }

        rc = i2c_read_timeout_bounded_us(imu->comm.i2c.instance, imu->comm.i2c.address, buf, len, false,
                                         MPU6050_I2C_TIMEOUT_US);
        if (rc < 0) {
            mpu6050_maybe_recover_timeout(imu, rc);
            return false;
        }
    }

    return true;
}

void mpu6050_init(struct imu_sensor *imu) {
    if (imu->protocol == IMU_PROTOCOL_I2C) {
        // MPU6050 usually runs at 400kHz, but my unit appears to init & read fine @1Mhz
        i2c_init(imu->comm.i2c.instance, SENSOR_I2C_BAUDRATE);
        gpio_set_function(imu->comm.i2c.sda_gpio, GPIO_FUNC_I2C);
        gpio_set_function(imu->comm.i2c.scl_gpio, GPIO_FUNC_I2C);
        gpio_pull_up(imu->comm.i2c.sda_gpio);
        gpio_pull_up(imu->comm.i2c.scl_gpio);
    } else {
        // MPU6050 is typically I2C only. If SPI is requested, we fail.
        imu->available = false;
        return;
    }

    sleep_ms(100); // Give it some time to power up

    LOG("IMU", "MPU6050 Resetting...\n");
    write_register(imu, REG_PWR_MGMT_1, 0x80); // Device Reset
    sleep_ms(100);
    write_register(imu, REG_PWR_MGMT_1, 0x01); // Wake up, use PLL with X axis gyroscope reference
    sleep_ms(10);

    imu->available = mpu6050_check_availability(imu);
    if (!imu->available) {
        LOG("IMU", "MPU6050 Device not found!\n");
        return;
    }

    // Configure Scales
    write_register(imu, REG_ACCEL_CONFIG, 0x10); // ±8g
    write_register(imu, REG_GYRO_CONFIG, 0x10);  // 1000 dps

    // Set scale factors based on config
    // ±8g: 4096 LSB/g
    // 1000 dps: 32.8 LSB/dps
    imu->accel_lsb_per_g = 4096.0f;
    imu->gyro_lsb_per_dps = 32.8f;
}

bool mpu6050_check_availability(struct imu_sensor *imu) {
    uint8_t id = 0;
    for (int i = 0; i < 5; i++) {
        id = read_register(imu, REG_WHO_AM_I);
        if (id == 0x68) {
            return true;
        }
        sleep_ms(10);
    }
    return false;
}

void mpu6050_read_raw(struct imu_sensor *imu, int16_t raw[6]) {
    uint8_t buffer[14] = {0};
    if (!read_registers(imu, REG_ACCEL_XOUT_H, buffer, 14)) {
        memset(raw, 0, sizeof(int16_t) * 6);
        return;
    }

    // MPU6050 outputs are Big-Endian
    raw[0] = (int16_t)(buffer[0] << 8 | buffer[1]); // ax
    raw[1] = (int16_t)(buffer[2] << 8 | buffer[3]); // ay
    raw[2] = (int16_t)(buffer[4] << 8 | buffer[5]); // az
    // buffer[6,7] is temperature
    raw[3] = (int16_t)(buffer[8] << 8 | buffer[9]);   // gx
    raw[4] = (int16_t)(buffer[10] << 8 | buffer[11]); // gy
    raw[5] = (int16_t)(buffer[12] << 8 | buffer[13]); // gz
}

int16_t mpu6050_read_temperature(struct imu_sensor *imu) {
    uint8_t buffer[2] = {0};
    if (!read_registers(imu, REG_TEMP_OUT_H, buffer, 2)) {
        return 0;
    }
    return (int16_t)(buffer[0] << 8 | buffer[1]);
}

float mpu6050_temperature_celsius(struct imu_sensor *imu) {
    int16_t raw_temperature = mpu6050_read_temperature(imu);
    // Formula from datasheet: T = (raw / 340) + 36.53
    return (float)raw_temperature / 340.0f + 36.53f;
}
