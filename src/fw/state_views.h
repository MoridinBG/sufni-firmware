#ifndef STATE_VIEWS_H
#define STATE_VIEWS_H

#include <stdbool.h>
#include <stdint.h>

#include "ssd1306.h"

struct idle_view_model {
    bool battery_power;
    uint8_t voltage_percentage;
    uint8_t hour;
    uint8_t minute;
    bool fork_available;
    bool shock_available;
    bool imu_frame_available;
    bool imu_fork_available;
};

void display_gps_wait_view(ssd1306_t *disp, bool fix_ready, uint8_t satellites, float epe);
void display_idle_view(ssd1306_t *disp, const struct idle_view_model *model);

#endif // STATE_VIEWS_H