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
    char timezone[100];
    uint32_t country;
    uint16_t travel_sample_rate;
    uint16_t imu_sample_rate;
    uint16_t gps_sample_rate;
    uint16_t temperature_period_seconds;
};

extern struct config config;

bool config_load_file(const char *path, struct config *out);
bool config_commit_staged_file(const char *staged_path);
void config_apply_snapshot(const struct config *snapshot);
bool load_config();

#endif // SUFNI_FW_UTIL_CONFIG_H
