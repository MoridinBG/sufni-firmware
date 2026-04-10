#include "helpers.h"

#include "hardware/adc.h"
#include "hardware/watchdog.h"
#include "pico/cyw43_arch.h"
#include "pico/time.h"
#include "tusb.h"

void soft_reset(void) {
    watchdog_enable(1, 1);
    while (1);
}

bool on_battery(void) {
    cyw43_thread_enter();
    bool ret = !cyw43_arch_gpio_get(2);
    cyw43_thread_exit();
    return ret;
}

float read_voltage(void) {
    cyw43_thread_enter();
    sleep_ms(1);
    adc_gpio_init(29);
    adc_select_input(3);
    uint32_t vsys = 0;
    for (int i = 0; i < 3; i++) { vsys += adc_read(); }
    cyw43_thread_exit();
    const float conversion_factor = 3.3f / (1 << 12);
    return vsys * conversion_factor;
}

bool msc_present(void) {
#ifdef USB_UART_DEBUG
    return false;
#else
    uint32_t t = time_us_32();
    while (!tud_ready()) {
        if (time_us_32() - t > 1000000) {
            return false;
        }
        tud_task();
    }
    return true;
#endif
}