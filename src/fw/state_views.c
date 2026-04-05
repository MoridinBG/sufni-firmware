#include "state_views.h"

#include <stdio.h>

void display_gps_wait_view(ssd1306_t *disp, bool fix_ready, uint8_t satellites, float epe) {
    ssd1306_clear(disp);

    if (fix_ready) {
        ssd1306_draw_string(disp, 0, 0, 2, "GPS OK");
        ssd1306_draw_string(disp, 0, 24, 1, "press to start");
    } else {
        char status[20];

        snprintf(status, sizeof(status), "SAT:%d EPE:%.1f", satellites, epe);
        ssd1306_draw_string(disp, 0, 0, 2, "GPS...");
        ssd1306_draw_string(disp, 0, 24, 1, status);
    }

    ssd1306_show(disp);
}

void display_idle_view(ssd1306_t *disp, const struct idle_view_model *model) {
    char battery_str[] = " PWR";
    char time_str[] = "00:00";

    if (model->battery_power) {
        if (model->voltage_percentage > 99) {
            snprintf(battery_str, sizeof(battery_str), "FULL");
        } else {
            snprintf(battery_str, sizeof(battery_str), "% 3d%%", model->voltage_percentage);
        }
    }

    snprintf(time_str, sizeof(time_str), "%02d:%02d", model->hour, model->minute);

    ssd1306_clear(disp);
    ssd1306_draw_string(disp, 96, 0, 1, battery_str);
    ssd1306_draw_string(disp, 0, 0, 2, time_str);

    if (model->fork_available) {
        ssd1306_draw_string(disp, 0, 24, 1, "fork");
    }
    if (model->shock_available) {
        ssd1306_draw_string(disp, 30, 24, 1, "shock");
    }
    if (model->imu_frame_available) {
        ssd1306_draw_string(disp, 63, 24, 1, "iFra");
    }
    if (model->imu_fork_available) {
        ssd1306_draw_string(disp, 90, 24, 1, "iFor");
    }

    ssd1306_show(disp);
}