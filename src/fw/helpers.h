#ifndef HELPERS_H
#define HELPERS_H

#include <stdbool.h>

void soft_reset(void);
bool on_battery(void);
float read_voltage(void);
bool msc_present(void);
bool wifi_connect(bool do_ntp);
void wifi_disconnect(void);

#endif // HELPERS_H