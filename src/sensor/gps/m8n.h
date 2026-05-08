#ifndef M8N_H
#define M8N_H

#include "gps_sensor.h"

// Sends raw bytes to the M8N UART for passthrough/debug command paths.
void m8n_send_command(struct gps_sensor *gps, const uint8_t *data, size_t len);

// Initializes the UART, UBX transport, autobaud recovery, and availability probe.
void m8n_init(struct gps_sensor *gps);

// Applies receiver policy for fix rate, constellation mix, NAV-PVT output, and persistence.
bool m8n_configure(struct gps_sensor *gps, uint16_t fix_interval_ms, bool gps_en, bool glonass_en, bool galileo_en,
                   bool bds_en, bool qzss_en);

// Drains buffered UART bytes through NMEA probe accounting and the UBX parser.
void m8n_process(struct gps_sensor *gps);

// Requests a UBX hot start without clearing long-lived receiver data.
bool m8n_hot_start(struct gps_sensor *gps);

// Requests a UBX cold start by clearing receiver navigation data.
bool m8n_cold_start(struct gps_sensor *gps);

// Wakes and probes the receiver, including baud recovery.
bool m8n_power_on(struct gps_sensor *gps);

// Sends UBX PMREQ backup-mode request and disables RX handling.
bool m8n_power_off(struct gps_sensor *gps);

// Enables UART RX interrupt capture for the shared GPS ring buffer.
void m8n_rx_enable(struct gps_sensor *gps);

// Disables UART RX interrupt capture for the shared GPS ring buffer.
void m8n_rx_disable(struct gps_sensor *gps);

// Reports whether M8N UART RX interrupt capture is currently enabled.
bool m8n_rx_is_enabled(void);

#endif
