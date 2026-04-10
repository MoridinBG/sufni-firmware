#include "wifi.h"

#include "dhcpserver.h"

#include "../ntp/ntp.h"
#include "../util/config.h"
#include "../util/log.h"

#include "pico/cyw43_arch.h"
#include "pico/time.h"

static bool wifi_running = false;
static enum wifi_mode wifi_active_mode = WIFI_MODE_STA;
static struct netif *active_netif = NULL;
static dhcp_server_t ap_dhcp_server;
static bool ap_dhcp_running = false;

static struct netif *wifi_netif_for_mode(enum wifi_mode mode) {
    return &cyw43_state.netif[mode == WIFI_MODE_AP ? CYW43_ITF_AP : CYW43_ITF_STA];
}

static bool wifi_start_sta(bool do_ntp) {
    LOG("WiFi", "Enabling STA mode\n");
    cyw43_arch_enable_sta_mode();

    LOG("WiFi", "Connecting to SSID: %s\n", config.sta_ssid);
    bool connected =
        cyw43_arch_wifi_connect_timeout_ms(config.sta_ssid, config.sta_psk, CYW43_AUTH_WPA2_AES_PSK, 20000) == 0;
    if (!connected) {
        LOG("WiFi", "STA connection failed\n");
        cyw43_arch_disable_sta_mode();
        sleep_ms(100);
        return false;
    }

    wifi_running = true;
    wifi_active_mode = WIFI_MODE_STA;
    active_netif = wifi_netif_for_mode(WIFI_MODE_STA);

    LOG("WiFi", "STA connected successfully\n");
    if (do_ntp) {
        LOG("WiFi", "Syncing RTC to NTP\n");
        sync_rtc_to_ntp();
    }

    return true;
}

static bool wifi_start_ap(void) {
    LOG("WiFi", "Enabling AP mode, SSID: %s\n", config.ap_ssid);
    cyw43_arch_enable_ap_mode(config.ap_ssid, config.ap_psk, CYW43_AUTH_WPA2_AES_PSK);

    struct netif *ap_netif = wifi_netif_for_mode(WIFI_MODE_AP);

    cyw43_arch_lwip_begin();
    ip_addr_t ap_ip;
    ip_addr_t ap_netmask;
    ip_addr_copy_from_ip4(ap_ip, *netif_ip4_addr(ap_netif));
    ip_addr_copy_from_ip4(ap_netmask, *netif_ip4_netmask(ap_netif));
    dhcp_server_init(&ap_dhcp_server, &ap_ip, &ap_netmask);
    ap_dhcp_running = ap_dhcp_server.udp != NULL;
    cyw43_arch_lwip_end();

    if (!ap_dhcp_running) {
        LOG("WiFi", "AP DHCP server failed to start\n");
        cyw43_arch_disable_ap_mode();
        sleep_ms(100);
        return false;
    }

    wifi_running = true;
    wifi_active_mode = WIFI_MODE_AP;
    active_netif = ap_netif;

    LOG("WiFi", "AP mode enabled with DHCP\n");
    return true;
}

bool wifi_start_from_config(bool do_ntp) {
    if (wifi_running) {
        wifi_stop();
    }

    active_netif = NULL;
    wifi_running = false;

    if (config.wifi_mode == WIFI_MODE_AP) {
        return wifi_start_ap();
    }
    return wifi_start_sta(do_ntp);
}

void wifi_stop(void) {
    if (!wifi_running) {
        active_netif = NULL;
        return;
    }

    LOG("WiFi", "Stopping WiFi\n");
    if (wifi_active_mode == WIFI_MODE_AP) {
        if (ap_dhcp_running) {
            cyw43_arch_lwip_begin();
            dhcp_server_deinit(&ap_dhcp_server);
            cyw43_arch_lwip_end();
            ap_dhcp_running = false;
        }
        cyw43_arch_disable_ap_mode();
    } else {
        cyw43_arch_disable_sta_mode();
    }

    sleep_ms(100);
    wifi_running = false;
    active_netif = NULL;
}

bool wifi_mode_is_ap(void) { return wifi_running && wifi_active_mode == WIFI_MODE_AP; }

struct netif *wifi_active_netif(void) { return active_netif; }