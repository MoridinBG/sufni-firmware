#include "hardware/gpio.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "pushbutton.h"
#include "../util/log.h"

// Tactile mechanical buttons typically bounce for a few milliseconds per transition,
// with the contact rapidly chattering between open and closed before settling. The
// debounce window must outlast the longest expected bounce burst with margin; 15 ms
// is well above typical bounce duration and still well below the threshold of human
// perception, so a press feels instantaneous.
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

// Fires once the line has been quiet for PUSHBUTTON_DEBOUNCE_US. A bounce burst that
// briefly reverts to the original level produces no dispatch because the read here
// matches stable_state.
static int64_t debounce_callback(alarm_id_t id, void *user_data) {
    struct button *btn = user_data;
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
        // Suppress the short-press dispatch when the long-press already fired, so a single
        // long hold yields exactly one event.
        if (!btn->longpress_fired && btn->onpress != NULL) {
            LOG("BUTTON", "Button %u press\n", btn->gpio);
            btn->onpress(btn->user_data);
        }
    }
    return 0;
}

// IRQ for any registered button pin. Runs in the GPIO IRQ context, so it must stay short
// and avoid blocking work; the actual decision is deferred to the alarm callback.
static void gpio_callback(uint gpio, uint32_t events) {
    (void)events;
    struct button *btn = buttons[gpio];
    if (NULL != btn && btn->enabled) {
        // Sliding window: each new edge resets the debounce countdown so the alarm only fires
        // once the line has been quiet for the full window.
        if (btn->debounce_alarm != -1) {
            cancel_alarm(btn->debounce_alarm);
        }
        btn->debounce_alarm = add_alarm_in_us(PUSHBUTTON_DEBOUNCE_US, debounce_callback, btn, true);
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

// Re-enable a previously disabled button. The first edge after this call schedules a
// fresh debounce window, and stable_state is resampled so a level change that happened
// while the button was disabled does not produce a phantom transition.
void enable_button(uint gpio) {
    struct button *btn = buttons[gpio];
    btn->stable_state = gpio_get(gpio);
    btn->longpress_fired = false;
    btn->enabled = true;
    gpio_set_irq_enabled(gpio, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
}
