#ifndef _PUSHBUTTON_H
#define _PUSHBUTTON_H

#include "pico/time.h"

struct button {
    uint gpio;
    bool stable_state; // true = released (pin high via pull-up), false = pressed
    bool enabled;
    bool longpress_fired;
    alarm_id_t debounce_alarm;
    alarm_id_t longpress_alarm;
    volatile uint64_t last_edge_us;
    void *user_data;
    void (*onpress)(void *user_data);
    void (*onlongpress)(void *user_data);
};

void create_button(uint gpio, void *user_data, void (*onpress)(void *), void (*onlongpress)(void *));
void disable_button(uint gpio, bool release_only);
void enable_button(uint gpio);

#endif /* _PUSHBUTTON_H */
