#include "hardware/gpio.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "pushbutton.h"
#include "../util/log.h"

// 15 ms outlasts typical contact bounce while staying below the threshold of human perception.
#define PUSHBUTTON_DEBOUNCE_US  15000
#define PUSHBUTTON_LONGPRESS_US 1000000

static struct button *buttons[28] = {NULL};

static int64_t longpress_callback(alarm_id_t id, void *user_data) {
    struct button *btn = user_data;
    btn->longpress_alarm = -1;
    if (btn->stable_state == false) {
        btn->longpress_fired = true;
        if (btn->onlongpress != NULL) {
            LOG("BUTTON", "Button %u long press\n", btn->gpio);
            btn->onlongpress(btn->user_data);
        }
    }
    return 0;
}

// Fires once the line has been quiet for PUSHBUTTON_DEBOUNCE_US; if a newer edge arrived
// meanwhile, returning a positive delta reuses this same slot for the next firing.
static int64_t debounce_callback(alarm_id_t id, void *user_data) {
    struct button *btn = user_data;
    uint64_t now = time_us_64();
    uint64_t since_last_edge = now - btn->last_edge_us;
    if (since_last_edge < PUSHBUTTON_DEBOUNCE_US) {
        return (int64_t)(PUSHBUTTON_DEBOUNCE_US - since_last_edge);
    }

    btn->debounce_alarm = -1;
    bool current = gpio_get(btn->gpio);
    if (current == btn->stable_state) {
        return 0;
    }

    btn->stable_state = current;
    if (current == false) {
        btn->longpress_fired = false;
        if (btn->longpress_alarm != -1) {
            cancel_alarm(btn->longpress_alarm);
        }
        btn->longpress_alarm = add_alarm_in_us(PUSHBUTTON_LONGPRESS_US, longpress_callback, btn, true);
    } else {
        if (btn->longpress_alarm != -1) {
            cancel_alarm(btn->longpress_alarm);
            btn->longpress_alarm = -1;
        }
        // Suppress the short-press dispatch when long-press already fired so a hold yields one event.
        if (!btn->longpress_fired && btn->onpress != NULL) {
            LOG("BUTTON", "Button %u press\n", btn->gpio);
            btn->onpress(btn->user_data);
        }
    }
    return 0;
}

// Runs in GPIO IRQ. We never cancel+re-add here: under a tail-chaining bounce burst
// cancel_alarm can't free slots until the timer IRQ runs, so each edge would consume a fresh slot.
static void gpio_callback(uint gpio, uint32_t events) {
    (void)events;
    struct button *btn = buttons[gpio];
    if (NULL != btn && btn->enabled) {
        btn->last_edge_us = time_us_64();
        if (btn->debounce_alarm == -1) {
            btn->debounce_alarm = add_alarm_in_us(PUSHBUTTON_DEBOUNCE_US, debounce_callback, btn, true);
        }
    }
}

void create_button(uint gpio, void *user_data, void (*onpress)(void *), void (*onlongpress)(void *)) {
    gpio_init(gpio);
    gpio_pull_up(gpio);
    gpio_set_irq_enabled_with_callback(gpio, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

    struct button *btn = malloc(sizeof(struct button)); // NOTE: we are never freeing them, but
                                                        // buttons are assumed to live forever.
    btn->gpio = gpio;
    btn->stable_state = gpio_get(gpio); // should be true, since we pulled the GPIO up
    btn->enabled = true;
    btn->longpress_fired = false;
    btn->debounce_alarm = -1;
    btn->longpress_alarm = -1;
    btn->user_data = user_data;
    btn->onpress = onpress;
    btn->onlongpress = onlongpress;
    buttons[gpio] = btn;
}

void disable_button(uint gpio, bool release_only) {
    struct button *btn = buttons[gpio];
    btn->enabled = false;
    if (btn->debounce_alarm != -1) {
        cancel_alarm(btn->debounce_alarm);
        btn->debounce_alarm = -1;
    }
    if (btn->longpress_alarm != -1) {
        cancel_alarm(btn->longpress_alarm);
        btn->longpress_alarm = -1;
    }
    uint32_t mask = GPIO_IRQ_EDGE_RISE | (release_only ? 0 : GPIO_IRQ_EDGE_FALL);
    gpio_set_irq_enabled(gpio, mask, false);
}

// Resample stable_state so a level change that happened while disabled does not produce a phantom transition.
void enable_button(uint gpio) {
    struct button *btn = buttons[gpio];
    btn->stable_state = gpio_get(gpio);
    btn->longpress_fired = false;
    btn->enabled = true;
    gpio_set_irq_enabled(gpio, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
}
