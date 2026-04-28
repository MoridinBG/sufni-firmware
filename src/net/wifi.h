#ifndef WIFI_H
#define WIFI_H

#include <stdbool.h>

#include "lwip/netif.h"

struct config;

bool wifi_start_with_config(const struct config *cfg, bool do_ntp);
void wifi_stop(void);
struct netif *wifi_active_netif(void);
bool wifi_client_connected(void);

#endif // WIFI_H