# Sufni Suspension Telemetry - Firmware

Pico W / Pico 2 W firmware for an MTB suspension telemetry DAQ. C11, Pico SDK 2.2.0, CMake build
system. Open source under MIT.

Use this file as a quick onboarding guide. For deep detail, read [ARCHITECTURE.md](ARCHITECTURE.md).
For wire formats, prefer the protocol headers and dedicated specs over summaries:

- `src/fw/sst.h` - SST file format source of truth
- `src/net/live_protocol.h` and [LIVE_STREAMING_SPECS.md](LIVE_STREAMING_SPECS.md) - live preview protocol
- `src/net/management_protocol.h` and [MANAGEMENT_PROTOCOL_SPECS.md](MANAGEMENT_PROTOCOL_SPECS.md) - management protocol

If documentation and code disagree, treat current code and protocol headers as authoritative.

## Quick orientation

- `src/fw/main.c` - Entry point, state machine, button actions, sleep/wake, recording and TCP-service transitions
- `src/fw/fw_init.c` - Boot/init workflow: board, display, storage, RTC/AON timer, CONFIG, sensors, Core 1 launch
- `src/fw/fw_state.h` - Shared state enum and volatile runtime flags
- `src/fw/core1_ipc.h` - FIFO message families, dispatcher/storage commands, shared mailbox helpers
- `src/fw/core1_worker.c` - Core 1 dispatcher; runs either the storage backend or TCP server backend
- `src/fw/data_acquisition.c` - Core 0 recording timers, sensor buffers, GPS callbacks, markers, storage FIFO commands
- `src/fw/data_storage.c` - Core 1 storage backend, SST file creation, chunk serialization, file close
- `src/fw/live_stream_shared.c` - Cross-core live stream state, slot pools, queue depth and drop counters
- `src/fw/live_core0_session.c` - Core 0 live session startup/teardown, negotiated timers, GPS routing, slot filling
- `src/fw/live_watchdog_diag.c` - Live stream diagnostics/watchdog logging
- `src/fw/state_views.c` - Display rendering for `IDLE` and `GPS_WAIT`
- `src/fw/sst.c` / `src/fw/sst.h` - SST structs, chunk types, metadata helpers
- `src/fw/hardware_config.h` - Pin assignments and compile-time hardware selection macros
- `src/fw/sensor_setup.c` - Compile-time-selected global GPS/IMU sensor instances
- `src/fw/calibration_flow.c` - Interactive travel and IMU calibration
- `src/fw/calibration_storage.c` - Binary calibration file read/write
- `src/sensor/` - Sensor drivers: travel (`AS5600`, linear ADC), IMU (`MPU6050`, `LSM6DSO`), GPS (`LC76G`, `M8N`/BN-880)
- `src/net/` - WiFi/AP/DHCP, TCP server, live protocol, management protocol
- `src/msc/` - USB Mass Storage mode (TinyUSB), disabled when `USB_UART_DEBUG=ON`
- `src/fs/` - MicroSD setup over SPI or SDIO
- `src/util/` - CONFIG parsing, logging, list helpers, safe I2C helpers
- `tools/` - SST inspection, SST-to-GPX extraction, OLED display mock rendering
- `generate_cmake_presets.py` - Interactive generator for hardware-specific `CMakePresets.json`

`CLAUDE.md` is a symlink to AGENTS.md

## Building

Use the configured presets in `CMakePresets.json`. They are generated for the local hardware setup by
`generate_cmake_presets.py`, so do not regenerate them unless the task explicitly asks for a new hardware config.

```sh
cmake --preset <preset-name>
cmake --build build/<preset-name>
```

The output is `sufni-suspension-telemetry.uf2` in the preset build directory, flashable via BOOTSEL or `picotool`.

## Key build variables

Configure through CMake presets or `-D` flags. See [ARCHITECTURE.md](ARCHITECTURE.md) for the full table.

- `PICO_BOARD`: `pico_w` or `pico2_w`
- `SPI_MICROSD`: `ON` for SPI SD card, `OFF` for SDIO
- `DISP_PROTO`: `PIO_I2C` or `SPI`
- `GPS_MODULE`: `NONE`, `LC76G`, or `M8N`
- `IMU_FRAME` / `IMU_FORK` / `IMU_REAR`: `NONE`, `MPU6050`, or `LSM6DSO`
- `IMU_FRAME_PROTO` / `IMU_FORK_PROTO` / `IMU_REAR_PROTO`: `I2C` or `SPI` where supported
- `FORK_LINEAR` / `SHOCK_LINEAR`: `ON` for ADC linear sensor, `OFF` for AS5600 rotational
- `USB_UART_DEBUG`: `ON` swaps USB MSC for USB serial debug output
- `LOG_TO_FILE`: `ON` enables SD-card logging in non-release debug builds

## Architecture cautions

- Dual-core ownership matters. Core 0 owns the state machine, buttons/display, recording timers, GPS routing, and live
  slot filling. Core 1 runs `core1_worker_main()` and switches between storage and TCP server backends.
- The multicore FIFO is a control mailbox only. It carries dispatcher/storage commands and acknowledgements, not live
  sample payloads.
- Live preview uses shared slot pools in `live_stream_shared`; Core 0 publishes `READY` slots, Core 1 drains and releases
  them. Keep the existing memory-barrier pattern when touching slot or mailbox state.
- Recording and live preview are mutually exclusive in the current firmware. Live preview is available from `SERVE_TCP`.
- The TCP server accepts one client at a time on port `1557`; initial magic bytes select either `LIVE` or `MGMT`.
- Management config upload stages `CONFIG.TMP`, validates it through `src/util/config.c`, commits it to `CONFIG`, and
  applies the snapshot. `SET_TIME_REQ` updates the AON timer and DS3231 through `set_system_time_utc()`.
- SST is a packed little-endian TLV format. Keep `src/fw/sst.h`, tools, and any protocol docs in sync when changing it.

## CONFIG file

The device reads a `CONFIG` file from the SD card at boot. It is `key=value`, one entry per line. Parsed in
`src/util/config.c`.

Supported keys include `WIFI_MODE`, `STA_SSID`, `STA_PSK`, `AP_SSID`, `AP_PSK`, `NTP_SERVER`, `COUNTRY`, `TIMEZONE`,
`TRAVEL_SAMPLE_RATE`, `IMU_SAMPLE_RATE`, `GPS_SAMPLE_RATE`, and `TEMPERATURE_PERIOD`. Legacy aliases `SSID` and `PSK`
map to station credentials.

`TIMEZONE` accepts either a POSIX TZ string or a name resolved through `zones.csv` on the SD card.

## Code style

- `.clang-format` is present (LLVM base, 4-space indent, 120 column limit).
- Logging uses `LOG(source, fmt, ...)`; release builds compile it out, debug builds print or log to file depending on
  `LOG_TO_FILE`.
- Sensor drivers use function-pointer structs with compile-time selection in `sensor_setup.c`; follow that pattern for new
  sensors.
- Avoid editing `external/` unless the task is explicitly about vendored dependencies.
- `plans/` contains design notes and planning artifacts. Do not treat it as more authoritative than current source,
  protocol headers, or the maintained specs unless a task targets those files.
