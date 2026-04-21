#include "i2c_safe.h"

#include "hardware/gpio.h"
#include "hardware/regs/i2c.h"
#include "pico/error.h"
#include "pico/time.h"

static void i2c_release_line(uint gpio) {
    gpio_set_dir(gpio, GPIO_IN);
    gpio_pull_up(gpio);
}

static void i2c_drive_line_low(uint gpio) {
    gpio_put(gpio, 0);
    gpio_set_dir(gpio, GPIO_OUT);
}

bool i2c_recover_bus(i2c_inst_t *i2c, uint sda_gpio, uint scl_gpio, uint baudrate, uint pulse_count, uint delay_us) {
    uint pulse;
    bool recovered;

    i2c_deinit(i2c);
    gpio_set_function(sda_gpio, GPIO_FUNC_SIO);
    gpio_set_function(scl_gpio, GPIO_FUNC_SIO);
    i2c_release_line(sda_gpio);
    i2c_release_line(scl_gpio);
    busy_wait_us_32(delay_us);

    for (pulse = 0; pulse < pulse_count && (!gpio_get(sda_gpio) || !gpio_get(scl_gpio)); ++pulse) {
        i2c_drive_line_low(scl_gpio);
        busy_wait_us_32(delay_us);
        i2c_release_line(scl_gpio);
        busy_wait_us_32(delay_us);
    }

    i2c_drive_line_low(sda_gpio);
    busy_wait_us_32(delay_us);
    i2c_release_line(scl_gpio);
    busy_wait_us_32(delay_us);
    i2c_release_line(sda_gpio);
    busy_wait_us_32(delay_us);

    recovered = gpio_get(sda_gpio) && gpio_get(scl_gpio);

    i2c_init(i2c, baudrate);
    gpio_set_function(sda_gpio, GPIO_FUNC_I2C);
    gpio_set_function(scl_gpio, GPIO_FUNC_I2C);
    gpio_pull_up(sda_gpio);
    gpio_pull_up(scl_gpio);

    return recovered;
}

static bool i2c_wait_for_write_available(i2c_inst_t *i2c, absolute_time_t deadline) {
    while (!i2c_get_write_available(i2c)) {
        if (time_reached(deadline)) {
            return false;
        }
        tight_loop_contents();
    }

    return true;
}

int i2c_write_timeout_bounded_us(i2c_inst_t *i2c, uint8_t addr, const uint8_t *src, size_t len, bool nostop,
                                 uint timeout_us) {
    absolute_time_t deadline;
    bool abort = false;
    bool timeout = false;
    uint32_t abort_reason = 0;
    int byte_ctr;
    int ilen = (int)len;

    if (addr >= 0x80 || len == 0) {
        return PICO_ERROR_GENERIC;
    }

    deadline = make_timeout_time_us(timeout_us);

    i2c->hw->enable = 0;
    i2c->hw->tar = addr;
    i2c->hw->enable = 1;

    for (byte_ctr = 0; byte_ctr < ilen; ++byte_ctr) {
        bool first = byte_ctr == 0;
        bool last = byte_ctr == ilen - 1;

        if (!i2c_wait_for_write_available(i2c, deadline)) {
            timeout = true;
            abort = true;
            break;
        }

        i2c->hw->data_cmd = ((first && i2c->restart_on_next) ? 1u : 0u) << I2C_IC_DATA_CMD_RESTART_LSB |
                            ((last && !nostop) ? 1u : 0u) << I2C_IC_DATA_CMD_STOP_LSB | *src++;

        do {
            if (time_reached(deadline)) {
                timeout = true;
                abort = true;
            }
            tight_loop_contents();
        } while (!timeout && !(i2c->hw->raw_intr_stat & I2C_IC_RAW_INTR_STAT_TX_EMPTY_BITS));

        if (!timeout) {
            abort_reason = i2c->hw->tx_abrt_source;
            if (abort_reason) {
                i2c->hw->clr_tx_abrt;
                abort = true;
            }

            if (abort || (last && !nostop)) {
                do {
                    if (time_reached(deadline)) {
                        timeout = true;
                        abort = true;
                    }
                    tight_loop_contents();
                } while (!timeout && !(i2c->hw->raw_intr_stat & I2C_IC_RAW_INTR_STAT_STOP_DET_BITS));

                if (!timeout) {
                    i2c->hw->clr_stop_det;
                }
            }
        }

        if (abort) {
            break;
        }
    }

    i2c->restart_on_next = nostop;

    if (abort) {
        if (timeout) {
            return PICO_ERROR_TIMEOUT;
        }

        if (!abort_reason || (abort_reason & I2C_IC_TX_ABRT_SOURCE_ABRT_7B_ADDR_NOACK_BITS)) {
            return PICO_ERROR_GENERIC;
        }

        if (abort_reason & I2C_IC_TX_ABRT_SOURCE_ABRT_TXDATA_NOACK_BITS) {
            return byte_ctr;
        }

        return PICO_ERROR_GENERIC;
    }

    return byte_ctr;
}

int i2c_read_timeout_bounded_us(i2c_inst_t *i2c, uint8_t addr, uint8_t *dst, size_t len, bool nostop, uint timeout_us) {
    absolute_time_t deadline;
    bool abort = false;
    bool timeout = false;
    uint32_t abort_reason = 0;
    int byte_ctr;
    int ilen = (int)len;

    if (addr >= 0x80 || len == 0) {
        return PICO_ERROR_GENERIC;
    }

    deadline = make_timeout_time_us(timeout_us);

    i2c->hw->enable = 0;
    i2c->hw->tar = addr;
    i2c->hw->enable = 1;

    for (byte_ctr = 0; byte_ctr < ilen; ++byte_ctr) {
        bool first = byte_ctr == 0;
        bool last = byte_ctr == ilen - 1;

        if (!i2c_wait_for_write_available(i2c, deadline)) {
            timeout = true;
            abort = true;
            break;
        }

        i2c->hw->data_cmd = ((first && i2c->restart_on_next) ? 1u : 0u) << I2C_IC_DATA_CMD_RESTART_LSB |
                            ((last && !nostop) ? 1u : 0u) << I2C_IC_DATA_CMD_STOP_LSB | I2C_IC_DATA_CMD_CMD_BITS;

        do {
            abort_reason = i2c->hw->tx_abrt_source;
            if (i2c->hw->raw_intr_stat & I2C_IC_RAW_INTR_STAT_TX_ABRT_BITS) {
                abort = true;
                i2c->hw->clr_tx_abrt;
            }

            if (time_reached(deadline)) {
                timeout = true;
                abort = true;
            }

            tight_loop_contents();
        } while (!abort && !i2c_get_read_available(i2c));

        if (abort) {
            break;
        }

        *dst++ = (uint8_t)i2c->hw->data_cmd;
    }

    i2c->restart_on_next = nostop;

    if (abort) {
        if (timeout) {
            return PICO_ERROR_TIMEOUT;
        }

        if (!abort_reason || (abort_reason & I2C_IC_TX_ABRT_SOURCE_ABRT_7B_ADDR_NOACK_BITS)) {
            return PICO_ERROR_GENERIC;
        }

        return PICO_ERROR_GENERIC;
    }

    return byte_ctr;
}