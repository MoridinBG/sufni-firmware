#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

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

static void reset_config_defaults(struct config *cfg) {
    *cfg = (struct config){
        .wifi_mode = WIFI_MODE_STA,
        .sta_ssid = "sst",
        .sta_psk = "changemeplease",
        .ap_ssid = "SufniDAQ",
        .ap_psk = "changemeplease",
        .ntp_server = "pool.ntp.org",
        .country = CYW43_COUNTRY_HUNGARY,
        .timezone = "UTC0",
    };
}

static void resolve_timezone_string(struct config *cfg, const char *tz) {
    copy_config_string(cfg->timezone, sizeof(cfg->timezone), tz);

    FIL zones_fil;
    FRESULT fr = f_open(&zones_fil, "zones.csv", FA_OPEN_EXISTING | FA_READ);
    if (fr == FR_OK || fr == FR_EXIST) {
        char line[128];
        while (f_gets(line, sizeof(line), &zones_fil) != NULL) {
            char *key = strtok(line, ",\"");
            char *value = strtok(NULL, ",\"");
            if (key == NULL || value == NULL) {
                continue;
            }
            value[strcspn(value, "\r\n")] = 0;
            if (strcmp(key, tz) == 0) {
                copy_config_string(cfg->timezone, sizeof(cfg->timezone), value);
                break;
            }
        }
    }
    f_close(&zones_fil);
}

static bool parse_wifi_mode_value(struct config *cfg, const char *value) {
    if (strcmp(value, "STA") == 0) {
        cfg->wifi_mode = WIFI_MODE_STA;
        return true;
    }
    if (strcmp(value, "AP") == 0) {
        cfg->wifi_mode = WIFI_MODE_AP;
        return true;
    }
    return false;
}

static bool parse_country_value(struct config *cfg, const char *value) {
    if (strlen(value) < 2) {
        return false;
    }
    cfg->country = CYW43_COUNTRY(value[0], value[1], 0);
    return true;
}

static bool validate_config(const struct config *cfg) {
    if (cfg->wifi_mode == WIFI_MODE_STA) {
        return cfg->sta_ssid[0] != 0 && cfg->sta_psk[0] != 0;
    }
    if (cfg->wifi_mode == WIFI_MODE_AP) {
        return cfg->ap_ssid[0] != 0 && strlen(cfg->ap_psk) >= 8;
    }
    return false;
}

bool config_load_file(const char *path, struct config *out) {
    FIL config_fil;
    struct config parsed;
    bool parse_ok = true;

    if (path == NULL || out == NULL) {
        return false;
    }

    reset_config_defaults(&parsed);

    FRESULT fr = f_open(&config_fil, path, FA_OPEN_EXISTING | FA_READ);
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
            parse_ok = parse_wifi_mode_value(&parsed, value) && parse_ok;
        } else if (strcmp(key, "SSID") == 0 || strcmp(key, "STA_SSID") == 0) {
            copy_config_string(parsed.sta_ssid, sizeof(parsed.sta_ssid), value);
        } else if (strcmp(key, "PSK") == 0 || strcmp(key, "STA_PSK") == 0) {
            copy_config_string(parsed.sta_psk, sizeof(parsed.sta_psk), value);
        } else if (strcmp(key, "AP_SSID") == 0) {
            copy_config_string(parsed.ap_ssid, sizeof(parsed.ap_ssid), value);
        } else if (strcmp(key, "AP_PSK") == 0) {
            copy_config_string(parsed.ap_psk, sizeof(parsed.ap_psk), value);
        } else if (strcmp(key, "NTP_SERVER") == 0) {
            copy_config_string(parsed.ntp_server, sizeof(parsed.ntp_server), value);
        } else if (strcmp(key, "COUNTRY") == 0) {
            parse_ok = parse_country_value(&parsed, value) && parse_ok;
        } else if (strcmp(key, "TIMEZONE") == 0) {
            resolve_timezone_string(&parsed, value);
        }
    }

    f_close(&config_fil);

    if (!parse_ok || !validate_config(&parsed)) {
        return false;
    }

    *out = parsed;
    return true;
}

bool config_commit_staged_file(const char *staged_path) {
    if (staged_path == NULL) {
        return false;
    }

    if (ff_rename(staged_path, "CONFIG", 1) != 0) {
        return false;
    }

    return true;
}

void config_apply_snapshot(const struct config *snapshot) {
    if (snapshot == NULL) {
        return;
    }

    config = *snapshot;
}

bool load_config() {
    struct config snapshot;

    reset_config_defaults(&config);
    if (!config_load_file("CONFIG", &snapshot)) {
        return false;
    }

    config_apply_snapshot(&snapshot);
    return true;
}
