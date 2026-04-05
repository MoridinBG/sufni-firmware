#include "display.h"

#include "../pio_i2c/pio_i2c.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "hardware_config.h"

void display_message(ssd1306_t *disp, const char *message) {
    ssd1306_clear(disp);
    ssd1306_draw_string(disp, 0, 10, 2, message);
    ssd1306_show(disp);
}

void setup_display(ssd1306_t *disp) {
#ifdef SPI_DISPLAY
    spi_init(DISPLAY_SPI, 1000000);
    gpio_set_function(DISPLAY_PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(DISPLAY_PIN_MOSI, GPIO_FUNC_SPI);

    disp->external_vcc = false;
    ssd1306_proto_t p = {
        DISPLAY_SPI,
        DISPLAY_PIN_CS,
        DISPLAY_PIN_MISO,
        DISPLAY_PIN_RST,
    };
    ssd1306_init(disp, DISPLAY_WIDTH, DISPLAY_HEIGHT, p);
#else
    ssd1306_proto_t p = {DISPLAY_ADDRESS, I2C_PIO, I2C_SM, pio_i2c_write_blocking};
    ssd1306_init(disp, DISPLAY_WIDTH, DISPLAY_HEIGHT, p);
#endif

    ssd1306_flip(disp, DISPLAY_FLIPPED);
    ssd1306_clear(disp);
    ssd1306_show(disp);
}