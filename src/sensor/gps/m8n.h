#ifndef M8N_H
#define M8N_H

#include "gps_sensor.h"

void m8n_send_command(struct gps_sensor *gps, const uint8_t *data, size_t len);

void m8n_init(struct gps_sensor *gps);
bool m8n_configure(struct gps_sensor *gps, uint16_t fix_interval_ms, bool gps_en, bool glonass_en, bool galileo_en,
                   bool bds_en, bool qzss_en);
void m8n_process(struct gps_sensor *gps);
bool m8n_hot_start(struct gps_sensor *gps);
bool m8n_cold_start(struct gps_sensor *gps);
bool m8n_power_on(struct gps_sensor *gps);
bool m8n_power_off(struct gps_sensor *gps);

void m8n_rx_enable(struct gps_sensor *gps);
void m8n_rx_disable(struct gps_sensor *gps);
bool m8n_rx_is_enabled(void);

#endif
