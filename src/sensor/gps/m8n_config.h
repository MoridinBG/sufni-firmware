#ifndef M8N_CONFIG_H
#define M8N_CONFIG_H

#include "ubx.h"

// UBX I/O hooks used by the M8N configuration policy.
// The service callback must drain RX bytes often enough for ACK and poll waits.
struct m8n_ubx_io {
    struct gps_sensor *gps;
    struct ubx_transport *ubx;
    ubx_service_fn service;
    void *service_context;
};

// Polls CFG-RATE to verify that the receiver is responding at the active baud.
bool m8n_config_probe_link(const struct m8n_ubx_io *io);

// Writes CFG-PRT for UART1 to switch the receiver to target_baud.
bool m8n_config_set_uart_baud(const struct m8n_ubx_io *io, uint target_baud);

// Applies the M8N runtime configuration: NAV5, GNSS mix, rate, messages, and save.
bool m8n_configure_receiver(const struct m8n_ubx_io *io, uint16_t fix_interval_ms, bool gps_en, bool glonass_en,
                            bool galileo_en, bool bds_en, bool qzss_en);

#endif
