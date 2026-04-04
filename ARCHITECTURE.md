# Sufni Suspension Telemetry - Firmware Architecture

Firmware for a Pico W / Pico 2 W based mountain bike suspension telemetry data acquisition unit. Part of the wider Sufni open-source project (MIT license). Records sensor data to SD card in a custom TLV binary format (SST). Desktop and mobile applications consume these files for visualization and analysis.

## High-level overview

```
                Core 0                          Core 1
        ┌─────────────────────┐        ┌──────────────────┐
        │  State machine      │  FIFO  │  SD card writer  │
        │  Sensor sampling    │───────>│  (data_storage_  │
        │  (timer callbacks)  │<───────│   core1)         │
        │  Button handling    │        └──────────────────┘
        │  Display updates    │
        │  WiFi / TCP / NTP   │
        └─────────────────────┘
```

The firmware uses both RP2040/RP2350 cores. Core 0 runs the main state machine, sensor sampling via repeating timer callbacks, and all I/O (display, buttons, WiFi). Core 1 is dedicated to writing data to the SD card. The two cores communicate via the Pico's hardware multicore FIFO using a command protocol (see Data pipeline).

## State machine

The main loop (`main.c:1340`) dispatches to a handler function indexed by the current state:

| State | Trigger | Description |
|---|---|---|
| `IDLE` | Default after init | Shows clock, battery, sensor status on display. Polls sensor availability. |
| `SLEEP` | Right press in IDLE | Deep sleep (WFI). Saves power by stopping clocks and display. |
| `WAKING` | Button interrupt from SLEEP | Restores clocks, display, buttons. Transitions to IDLE. |
| `REC_START` | Left press in IDLE | Applies calibration, inits sensors. If GPS available, goes to GPS_WAIT; otherwise starts recording. |
| `GPS_WAIT` | GPS available during REC_START | Waits for reliable GPS fix (10 consecutive good fixes with 3D fix, >=6 sats, EPE<=6m). User can skip (left press) or confirm when ready. |
| `RECORD` | After REC_START/GPS_WAIT | Active data acquisition. Timer callbacks sample sensors into double buffers. |
| `REC_STOP` | Left press in RECORD | Stops timers, flushes remaining data, closes file. |
| `SYNC_DATA` | Left long-press in IDLE | Connects WiFi, pushes all SST files to configured server via TCP client, moves to `uploaded/`. |
| `SERVE_TCP` | Right long-press in IDLE | Connects WiFi, starts TCP server with mDNS discovery. Clients can list/download/trash SST files. |
| `MSC` | USB cable detected at boot | USB Mass Storage mode. SD card exposed directly to host. Mutually exclusive with normal operation. |

Button mapping:
- **Left press**: Start/stop recording, confirm GPS, skip GPS wait
- **Left long-press**: Sync data to server
- **Right press**: Sleep (from IDLE), set marker (during recording), stop TCP server
- **Right long-press**: Start TCP server

## Data pipeline

### Double-buffer scheme

Each sensor type uses two statically allocated buffers. Core 0 fills the active buffer via timer callbacks. When full, the active buffer pointer is sent to Core 1 via FIFO, and Core 0 immediately switches to the other buffer.

```
Core 0 (sampling)                    Core 1 (writing)
  fill buffer A                        idle
  buffer A full ──FIFO──>             receive buffer A ptr
  switch to buffer B                   ack via FIFO
  fill buffer B                        write buffer A to SD
  buffer B full ──FIFO──>             receive buffer B ptr
  switch to buffer A                   ack via FIFO
  ...                                  write buffer B to SD
```

FIFO protocol - Core 0 pushes a command enum followed by command-specific arguments:

| Command | Arguments (pushed after command) | Description |
|---|---|---|
| `OPEN` | (none) | Open new SST file. Core 1 replies with file index, then initial buffer pointers. |
| `DUMP_TRAVEL` | `size`, `buffer_ptr` | Write travel chunk. Core 1 acks with buffer ptr before writing (non-blocking for Core 0). |
| `DUMP_IMU` | `size`, `buffer_ptr` | Write IMU chunk. Same ack pattern. |
| `DUMP_GPS` | `size`, `buffer_ptr` | Write GPS chunk. Same ack pattern. |
| `MARKER` | (none) | Write a zero-length marker chunk to the file. |
| `FINISH` | `travel_size`, `travel_ptr`, [`imu_size`, `imu_ptr`] | Flush remaining data and close file. |

### Sample rates and buffer sizes

- **Travel**: 1000 Hz, buffer 2048 records (4 bytes each = 8 KB)
- **IMU**: 1000 Hz, buffer scales with IMU count: `(IMU_COUNT + 1) * 512` records (12 bytes each)
- **GPS**: Fix interval 300ms, poll interval 200ms, buffer 30 records (46 bytes each). GPS data flows through a callback (`on_gps_fix`) rather than a dedicated timer-fill pattern; the callback writes directly to the GPS buffer during `RECORD` state.

## SST binary file format (v4)

The SST format is a Type-Length-Value encoded binary format. **This firmware is the source of truth for the format specification.** All structs are packed and little-endian.

### File header (16 bytes)

| Offset | Size | Field | Description |
|---|---|---|---|
| 0 | 3 | `magic` | Always `"SST"` |
| 3 | 1 | `version` | Currently `4` |
| 4 | 4 | `padding` | Reserved, zero |
| 8 | 8 | `timestamp` | `time_t` UTC epoch of recording start |

### Chunk header (3 bytes, packed)

| Offset | Size | Field | Description |
|---|---|---|---|
| 0 | 1 | `type` | `CHUNK_TYPE_*` identifier |
| 1 | 2 | `length` | Payload length in bytes (not including this header) |

### Chunk types

| Type | ID | Record size | Description |
|---|---|---|---|
| `RATES` | `0x00` | 3 bytes | Sample rate declarations. Must immediately follow the file header. One `samplerate_record` per sensor type: `{uint8_t type, uint16_t rate_hz}` |
| `TRAVEL` | `0x01` | 4 bytes | Suspension travel data. Array of `{uint16_t fork_angle, uint16_t shock_angle}` |
| `MARKER` | `0x02` | 0 bytes | User-placed marker (zero-length payload). Splits recording for segment analysis. |
| `IMU` | `0x03` | 12 bytes | 6-axis IMU data. Array of `{int16_t ax, ay, az, gx, gy, gz}`. Records interleave all active IMUs in meta order (frame, fork, rear). |
| `IMU_META` | `0x04` | 9 bytes per entry + 1 byte count | IMU metadata. Precedes all IMU data chunks. First byte is active IMU count, followed by that many `{uint8_t location_id, float accel_lsb_per_g, float gyro_lsb_per_dps}`. Location IDs: 0=Frame, 1=Fork, 2=Rear. |
| `GPS` | `0x05` | 46 bytes | GPS fix data. `{uint32_t date, uint32_t time_ms, double lat, double lon, float alt, float speed, float heading, uint8_t fix_mode, uint8_t satellites, float epe_2d, float epe_3d}` |

### File layout convention

```
[SST Header]
[RATES chunk]           ← one per file, contains travel + IMU sample rates
[IMU_META chunk]        ← one per file if IMUs present, defines IMU order
[TRAVEL chunks...]      ← bulk of the file
[IMU chunks...]         ← interleaved with travel chunks
[GPS chunks...]         ← appended when GPS buffer fills or on fix callback
[MARKER chunks...]      ← inserted at user request, appear between data chunks
```

## Sensor abstraction

All sensor types use a common pattern: a struct with function pointers set at compile time via designated initializers, selected by `#ifdef` / preprocessor conditionals. This gives polymorphism without runtime overhead.

### Travel sensors (`src/sensor/travel/`)

Interface: `struct travel_sensor` with `init`, `check_availability`, `start`, `measure`, `calibrate_expanded`, `calibrate_compressed`.

| Implementation | File | Hardware | Comm |
|---|---|---|---|
| AS5600 rotational | `rotational.c` | AS5600 magnetic rotary encoder | I2C (hardware) |
| Linear potentiometer | `linear.c` | ADC-based linear sensor | ADC |

The global `fork_sensor` and `shock_sensor` instances are defined in the matching implementation file, selected by `FORK_LINEAR` / `SHOCK_LINEAR` compile flags.

### IMU sensors (`src/sensor/imu/`)

Interface: `struct imu_sensor` with `init`, `check_availability`, `read_raw`, `read_temperature`, `temperature_celsius`.

| Implementation | File | Comm |
|---|---|---|
| MPU6050 | `mpu6050.c` | I2C only |
| LSM6DSO | `lsm6dso.c` | I2C or SPI |

Up to 3 IMU positions: **Frame** (on bike frame), **Fork** (on fork lowers), **Rear** (on rear triangle). Each position's chip and protocol are selected independently via cmake variables.

The `imu_sensor.c` layer adds:
- **Two-phase calibration**: stationary (gyro bias, gravity vector) then tilted (forward direction from gravity shift). Builds a 3x3 rotation matrix mapping sensor frame to bike frame (X=forward, Y=left, Z=up).
- **Temperature compensation**: Applies drift correction based on temperature difference from calibration point.
- **Coordinate transform**: All readings are rotated to bike frame before recording.

### GPS sensor (`src/sensor/gps/`)

Interface: `struct gps_sensor` with `init`, `configure`, `process`, `send_command`, `hot_start`, `cold_start`, `power_on`, `power_off`.

| Implementation | File | Comm |
|---|---|---|
| Quectel LC76G | `lc76g.c` | UART with IRQ-driven ring buffer |

Uses a **forked lwgps** library configured to parse Quectel proprietary PQTM messages (PVT + EPE) instead of standard NMEA. Standard NMEA output is disabled during configuration to reduce UART traffic.

The `gps_sensor.c` layer implements a **fix quality tracker**: requires 10 consecutive 3D fixes with >=6 satellites and EPE<=6m. Bad fixes roll back the counter by 3 (hysteresis to avoid oscillation).

GPS is power-managed: powered off after initial configuration, powered on when entering GPS_WAIT, powered off again when recording stops.

## Calibration system (`src/fw/calibration_*.c`)

Calibration data is stored in a `CALIBRATION` file on the SD card using a simple magic-byte-delimited binary format.

### Calibration flow

Triggered on boot if the `CALIBRATION` file is missing or left button is held:

1. **Travel expanded**: User extends suspension fully. Records AS5600 zero position / ADC baseline.
2. **Travel compressed**: User compresses suspension fully. Determines rotation direction (inverted flag). Saves travel calibration (creates `CALIBRATION` file).
3. **IMU stationary**: Bike level on ground. Averages 100 samples for gyro bias and gravity vector. Records temperature.
4. **IMU tilted**: Bike tilted front-up (on rear tire). Computes forward direction from gravity shift. Builds rotation matrix. Appends IMU calibration to file.

### Storage format

```
[T] travel_cal_data (7 bytes)
[I] imu_cal_data    (51 bytes, optional - frame IMU)
[F] imu_cal_data    (51 bytes, optional - fork IMU)
[R] imu_cal_data    (51 bytes, optional - rear IMU)
```

## Connectivity

### WiFi

Uses the CYW43 WiFi chip on Pico W / Pico 2 W. Configuration (SSID, PSK, country, NTP server, etc.) is read from a `CONFIG` file on the SD card (key=value format).

### NTP time sync

On WiFi connect, syncs time from a configurable NTP server via SNTP. Updates both the Pico's always-on timer and the external DS3231 RTC. Timezone handling uses POSIX TZ strings loaded from a `zones.csv` file on the SD card.

### TCP client (data sync)

`on_sync_data()` connects to a configured server (`sst_server:sst_server_port`), pushes each SST file with a header containing board ID + file size + filename. Successfully sent files are moved to `uploaded/`.

### TCP server (remote access)

Listens on port 1557 with mDNS service `_gosst._tcp`. Supports three operations:
- **Directory listing** (file ID 0): Returns board ID, sample rate, and metadata (name, size, timestamp) for all SST files.
- **File download** (file ID > 0): Streams the requested SST file, then moves it to `uploaded/`.
- **File trash** (file ID < 0): Moves the file to `trash/`.

### USB Mass Storage (MSC)

When USB is connected at boot, the device enters MSC mode and exposes the SD card as a USB mass storage device. Uses TinyUSB. Mutually exclusive with normal operation (MSC disables stdio USB; debug builds use `USB_UART_DEBUG` to swap MSC for USB serial).

## Hardware interfaces

### PIO I2C

A PIO-based I2C implementation (`src/pio_i2c/`) on PIO0 SM0 is used for the DS3231 RTC and I2C display. This frees up both hardware I2C peripherals for sensors.

### Pin allocation (default)

| Function | Pins | Interface |
|---|---|---|
| PIO I2C (RTC, display) | GP2 (SDA), GP3 (SCL) | PIO0 |
| Buttons | GP4 (left), GP5 (right) | GPIO IRQ |
| Fork sensor (rotational) | GP8 (SDA), GP9 (SCL) | I2C0 |
| Fork sensor (linear) | GP26 (ADC0) | ADC |
| Display (SPI variant) | GP10-13 | SPI1 |
| Shock sensor (rotational) | GP14 (SDA), GP15 (SCL) | I2C1 |
| MicroSD (SPI) | GP16-19 | SPI0 |
| MicroSD (SDIO) | GP17-22 | PIO1 |
| GPS (LC76G) | GP0 (TX), GP1 (RX) | UART0 |
| IMU (I2C, various positions) | Shares I2C0/I2C1 with travel sensors | I2C |
| IMU (SPI variant) | GP10-13 | SPI1 |
| Battery voltage | GP29 (ADC3) | ADC (VSYS/3) |

## Build system

### CMake presets

The firmware supports many hardware configurations via cmake cache variables. `generate_cmake_presets.py` is an interactive script that generates `CMakePresets.json` with the user's chosen configuration.

### Key cmake variables

| Variable | Values | Description |
|---|---|---|
| `PICO_BOARD` | `pico_w`, `pico2_w` | Target board |
| `SPI_MICROSD` | `ON`/`OFF` | SPI vs SDIO for SD card |
| `DISP_PROTO` | `PIO_I2C`, `SPI` | Display connection |
| `FORK_LINEAR` | `ON`/`OFF` | Linear (ADC) vs rotational (AS5600) fork sensor |
| `SHOCK_LINEAR` | `ON`/`OFF` | Linear (ADC) vs rotational (AS5600) shock sensor |
| `GPS_MODULE` | `NONE`, `LC76G` | GPS module selection |
| `IMU_FRAME` | `NONE`, `MPU6050`, `LSM6DSO` | Frame IMU chip |
| `IMU_FORK` | `NONE`, `MPU6050`, `LSM6DSO` | Fork IMU chip |
| `IMU_REAR` | `NONE`, `MPU6050`, `LSM6DSO` | Rear IMU chip |
| `IMU_*_PROTO` | `I2C`, `SPI` | IMU communication protocol (per position) |
| `USB_UART_DEBUG` | `ON`/`OFF` | Swap MSC for USB serial debug output |
| `CMAKE_BUILD_TYPE` | `Release`, `Debug` | Debug enables printf logging |

### External dependencies

| Library | Path | Purpose |
|---|---|---|
| no-OS-FatFS-SD-SDIO-SPI-RPi-Pico | `external/` | FAT filesystem + SD card driver (SPI and SDIO) |
| pico-as5600 | `external/` | AS5600 rotary encoder driver |
| pico-ssd1306 | `external/` | SSD1306 OLED display driver (I2C and SPI) |
| lwgps | `external/` | Lightweight GPS NMEA parser (forked for Quectel PQTM support) |
| Pico SDK 2.2.0 | System | RP2040/RP2350 HAL, WiFi, lwIP, TinyUSB |
| Pico Extras | System | Sleep, AON timer |

## SD card filesystem layout

```
/
├── CONFIG              Key=value config (SSID, PSK, NTP_SERVER, SST_SERVER, etc.)
├── CALIBRATION         Binary calibration data (travel + IMU)
├── BOARDID             Board unique ID string
├── INDEX               2-byte binary file counter
├── zones.csv           POSIX timezone string lookup table
├── 00001.SST           Recording files (5-digit zero-padded, .SST extension)
├── 00002.SST
├── ...
├── uploaded/           Successfully synced files moved here
└── trash/              Files trashed via TCP server moved here
```

## Test utilities

- `test_utils/inspect_sst_data.py`: Parses an SST file and prints chunk summary (types, counts, record counts, sample rates).
- `test_utils/sst_to_gpx.py`: Extracts GPS chunks from an SST file and generates a GPX track file with Garmin and custom SST extensions.
