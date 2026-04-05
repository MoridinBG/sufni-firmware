#ifndef DISPLAY_H
#define DISPLAY_H

#include "ssd1306.h"

void display_message(ssd1306_t *disp, const char *message);
void setup_display(ssd1306_t *disp);

#endif // DISPLAY_H