#ifndef FW_INIT_H
#define FW_INIT_H

#include <stdint.h>

#include "fw_state.h"
#include "calibration_flow.h"

#include "ssd1306.h"

#include "../rtc/ds3231.h"

struct fw_power_state {
    uint32_t scb_orig;
    uint32_t clock0_orig;
    uint32_t clock1_orig;
};

struct fw_button_handlers {
    void (*on_left_press)(void *user_data);
    void (*on_left_longpress)(void *user_data);
    void (*on_right_press)(void *user_data);
    void (*on_right_longpress)(void *user_data);
};

enum state fw_init(ssd1306_t *disp, struct ds3231 *rtc, struct calibration_ctx *cal_ctx,
                   struct fw_power_state *power_state, const struct fw_button_handlers *button_handlers);

#endif // FW_INIT_H