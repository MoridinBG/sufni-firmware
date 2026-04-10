#ifndef SUFNI_FW_UTIL_CONFIG_H
#define SUFNI_FW_UTIL_CONFIG_H

#include <stdbool.h>
#include <stdint.h>

enum wifi_mode {
    WIFI_MODE_STA,
    WIFI_MODE_AP,
};

struct config {
    enum wifi_mode wifi_mode;
    char sta_ssid[33];
    char sta_psk[64];
    char ap_ssid[33];
    char ap_psk[64];
    char ntp_server[264];
    char sst_server[264];
    char timezone[100];
    uint16_t sst_server_port;
    uint32_t country;
};

extern struct config config;

bool load_config();

#endif // SUFNI_FW_UTIL_CONFIG_H
