#ifndef I2C_SAFE_H
#define I2C_SAFE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "hardware/i2c.h"

bool i2c_recover_bus(i2c_inst_t *i2c, uint sda_gpio, uint scl_gpio, uint baudrate, uint pulse_count, uint delay_us);
int i2c_write_timeout_bounded_us(i2c_inst_t *i2c, uint8_t addr, const uint8_t *src, size_t len, bool nostop,
                                 uint timeout_us);
int i2c_read_timeout_bounded_us(i2c_inst_t *i2c, uint8_t addr, uint8_t *dst, size_t len, bool nostop, uint timeout_us);

#endif // I2C_SAFE_H