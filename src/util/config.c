#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "cyw43_country.h"

#include "config.h"
#include "ff_stdio.h"

struct config config = {
    .wifi_mode = WIFI_MODE_STA,
    .sta_ssid = "sst",
    .sta_psk = "changemeplease",
    .ap_ssid = "SufniDAQ",
    .ap_psk = "changemeplease",
    .ntp_server = "pool.ntp.org",
    .sst_server = "sst.sghctoma.com",
    .sst_server_port = 557,
    .country = CYW43_COUNTRY_HUNGARY,
    .timezone = "UTC0",
};

static void copy_config_string(char *dest, size_t dest_size, const char *src) {
    if (dest_size == 0) {
        return;
    }
    strncpy(dest, src, dest_size - 1);
    dest[dest_size - 1] = 0;
}

static void reset_config_defaults(void) {
    config = (struct config){
        .wifi_mode = WIFI_MODE_STA,
        .sta_ssid = "sst",
        .sta_psk = "changemeplease",
        .ap_ssid = "SufniDAQ",
        .ap_psk = "changemeplease",
        .ntp_server = "pool.ntp.org",
        .sst_server = "sst.sghctoma.com",
        .sst_server_port = 557,
        .country = CYW43_COUNTRY_HUNGARY,
        .timezone = "UTC0",
    };
}

static void get_tzstring(const char *tz) {
    copy_config_string(config.timezone, sizeof(config.timezone), tz);

    FIL zones_fil;
    FRESULT fr = f_open(&zones_fil, "zones.csv", FA_OPEN_EXISTING | FA_READ);
    if (fr == FR_OK || fr == FR_EXIST) {
        char line[128]; // longest line in the  2023c-2 is 65 characters long
        while (f_gets(line, 256, &zones_fil) != NULL) {
            char *key = strtok(line, ",\"");
            char *value = strtok(NULL, ",\"");
            value[strcspn(value, "\r\n")] = 0;
            if (key == NULL || value == NULL) {
                continue;
            }
            if (strcmp(key, tz) == 0) {
                copy_config_string(config.timezone, sizeof(config.timezone), value);
                break;
            }
        }
    }
    f_close(&zones_fil);
}

static bool parse_wifi_mode_value(const char *value) {
    if (strcmp(value, "STA") == 0) {
        config.wifi_mode = WIFI_MODE_STA;
        return true;
    }
    if (strcmp(value, "AP") == 0) {
        config.wifi_mode = WIFI_MODE_AP;
        return true;
    }
    return false;
}

static bool parse_country_value(const char *value) {
    if (strlen(value) < 2) {
        return false;
    }
    config.country = CYW43_COUNTRY(value[0], value[1], 0);
    return true;
}

static bool parse_port_value(const char *value) {
    char *end = NULL;
    unsigned long port = strtoul(value, &end, 10);
    if (end == value || *end != 0 || port > UINT16_MAX) {
        return false;
    }
    config.sst_server_port = (uint16_t)port;
    return true;
}

static bool validate_config(void) {
    if (config.wifi_mode == WIFI_MODE_STA) {
        return config.sta_ssid[0] != 0 && config.sta_psk[0] != 0;
    }
    if (config.wifi_mode == WIFI_MODE_AP) {
        return config.ap_ssid[0] != 0 && strlen(config.ap_psk) >= 8;
    }
    return false;
}

bool load_config() {
    FIL config_fil;
    bool parse_ok = true;

    reset_config_defaults();

    FRESULT fr = f_open(&config_fil, "CONFIG", FA_OPEN_EXISTING | FA_READ);
    if (!(fr == FR_OK || fr == FR_EXIST)) {
        return false;
    }

    char line[300];
    while (f_gets(line, 300, &config_fil) != NULL) {
        char *key = strtok(line, "=");
        char *value = strtok(NULL, "=");

        if (key == NULL || value == NULL) {
            continue;
        }

        value[strcspn(value, "\r\n")] = 0;

        if (strcmp(key, "WIFI_MODE") == 0) {
            parse_ok = parse_wifi_mode_value(value) && parse_ok;
        } else if (strcmp(key, "SSID") == 0 || strcmp(key, "STA_SSID") == 0) {
            copy_config_string(config.sta_ssid, sizeof(config.sta_ssid), value);
        } else if (strcmp(key, "PSK") == 0 || strcmp(key, "STA_PSK") == 0) {
            copy_config_string(config.sta_psk, sizeof(config.sta_psk), value);
        } else if (strcmp(key, "AP_SSID") == 0) {
            copy_config_string(config.ap_ssid, sizeof(config.ap_ssid), value);
        } else if (strcmp(key, "AP_PSK") == 0) {
            copy_config_string(config.ap_psk, sizeof(config.ap_psk), value);
        } else if (strcmp(key, "NTP_SERVER") == 0) {
            copy_config_string(config.ntp_server, sizeof(config.ntp_server), value);
        } else if (strcmp(key, "SST_SERVER") == 0) {
            copy_config_string(config.sst_server, sizeof(config.sst_server), value);
        } else if (strcmp(key, "SST_SERVER_PORT") == 0) {
            parse_ok = parse_port_value(value) && parse_ok;
        } else if (strcmp(key, "COUNTRY") == 0) {
            parse_ok = parse_country_value(value) && parse_ok;
        } else if (strcmp(key, "TIMEZONE") == 0) {
            get_tzstring(value);
        }
    }

    f_close(&config_fil);

    return parse_ok && validate_config();
}
