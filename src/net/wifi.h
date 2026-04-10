#ifndef WIFI_H
#define WIFI_H

#include <stdbool.h>

#include "lwip/netif.h"

bool wifi_start_from_config(bool do_ntp);
void wifi_stop(void);
bool wifi_mode_is_ap(void);
struct netif *wifi_active_netif(void);

#endif // WIFI_H