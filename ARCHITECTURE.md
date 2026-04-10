# Sufni Suspension Telemetry - Firmware Architecture

Firmware for a Pico W / Pico 2 W based mountain bike suspension telemetry data acquisition unit. Part of the wider Sufni open-source project (MIT license). Records sensor data to SD card in a custom TLV binary format (SST) and can also expose live preview data over TCP. Desktop and mobile applications consume these files and streams for visualization and analysis.

## High-level overview

```
Core 0                                      FIFO + Shared RAM                     Core 1
┌────────────────────────────┐                                             ┌────────────────────────────┐
│ State machine              │─ dispatch/storage control + status ───────>│ Dispatcher (`core1_worker`)│
│ Sensor sampling timers     │<──────────── buffer acks / events ─────────│ Storage backend            │
│ GPS parsing + calibration  │                                             │ TCP server backend         │
│ Display / buttons / WiFi   │══ live session state + slot pools ════════>│ Live framing + lwIP send   │
└────────────────────────────┘                                             └────────────────────────────┘
```

The firmware uses both RP2040/RP2350 cores. Core 0 owns the main state machine, sensor sampling via repeating timer callbacks, GPS UART processing, display, buttons, calibration application, and WiFi connect or disconnect lifecycle. Core 1 now runs a dispatcher that can enter either a storage session backend or a TCP server backend. The two cores use the Pico's hardware multicore FIFO as a control mailbox and use shared RAM for live-preview batch transport.

## Firmware module layout

| Module                                          | Responsibility                                                                                                              |
| ----------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------- |
| `main.c`                                        | State transitions, handler table dispatch, button actions, sleep/wake, top-level control flow                               |
| `fw_init.c` / `fw_init.h`                       | Boot path: board/display/storage/WiFi/RTC init, calibration setup/application, button registration, initial state selection |
| `fw_state.h`                                    | Shared `enum state` and volatile runtime flags used across modules                                                          |
| `core1_ipc.h`                                   | FIFO message-family definitions, dispatcher commands and events, storage session command namespace                          |
| `core1_worker.c` / `core1_worker.h`             | Core 1 dispatcher, backend mode selection, stop requests, and status tracking                                               |
| `data_acquisition.c` / `data_acquisition.h`     | Core 0 recording timers, acquisition buffers, GPS wait/record callbacks, marker flushing, and storage-session FIFO commands |
| `data_storage.c` / `data_storage.h`             | Core 1 storage session backend (`storage_session_run`), SST file creation, chunk serialization, and file close              |
| `data_sync.c` / `data_sync.h`                   | WiFi upload workflow for recorded SST files                                                                                 |
| `live_stream_shared.c` / `live_stream_shared.h` | Shared live session state, slot pools, queue counters, and control state used by both cores                                 |
| `live_stream_core0.c` / `live_stream_core0.h`   | Core 0 live-preview session start or stop, negotiated timers, GPS live routing, slot filling, and drop accounting           |
| `tcpserver.c` / `tcpserver.h`                   | Core 1-owned TCP backend that multiplexes file serving and the live protocol                                                |
| `live_stream_core1.c` / `live_stream_core1.h`   | Core 1 live control handling, frame emission, slot draining, teardown, and session stats                                    |
| `live_protocol.h`                               | Versioned wire format for live preview control, session metadata, batches, and stats                                        |
| `state_views.c` / `state_views.h`               | State-specific rendering for `IDLE` and `GPS_WAIT` screens                                                                  |
| `sensor_setup.c` / `sensor_setup.h`             | Compile-time-selected global GPS/IMU sensor instances                                                                       |
| `display.c` / `display.h`                       | Display setup and single-message helper                                                                                     |
| `helpers.c` / `helpers.h`                       | Shared runtime helpers such as WiFi, USB, battery, and reset helpers                                                        |

## State machine

The main loop in `src/fw/main.c` dispatches to a handler function indexed by the current state. Startup work is delegated to `fw_init.c`, acquisition to `data_acquisition.c`, sync to `data_sync.c`, and some state rendering to `state_views.c`.

| State       | Trigger                        | Description                                                                                                                                                 |
| ----------- | ------------------------------ | ----------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `IDLE`      | Default after init             | Shows clock, battery, sensor status on display. Polls sensor availability.                                                                                  |
| `SLEEP`     | Right press in IDLE            | Deep sleep (WFI). Saves power by stopping clocks and display.                                                                                               |
| `WAKING`    | Button interrupt from SLEEP    | Restores clocks, display, buttons. Transitions to IDLE.                                                                                                     |
| `REC_START` | Left press in IDLE             | Applies calibration, inits sensors. If GPS available, goes to GPS_WAIT; otherwise starts recording.                                                         |
| `GPS_WAIT`  | GPS available during REC_START | Waits for reliable GPS fix (10 consecutive good fixes with 3D fix, >=6 sats, EPE<=6m). User can skip (left press) or confirm when ready.                    |
| `RECORD`    | After REC_START/GPS_WAIT       | Active data acquisition. Repeating timer callbacks sample sensors into buffers while the main loop idles in the `RECORD` slot.                              |
| `REC_STOP`  | Left press in RECORD           | Stops timers, flushes remaining data, closes file.                                                                                                          |
| `SYNC_DATA` | Left long-press in IDLE        | Connects WiFi, pushes all SST files to configured server via TCP client, moves to `uploaded/`.                                                              |
| `SERVE_TCP` | Right long-press in IDLE       | Connects WiFi, requests the Core 1 TCP backend, and stays in a non-blocking coordination loop while Core 1 serves either the file protocol or live preview. |
| `MSC`       | USB cable detected at boot     | USB Mass Storage mode. SD card exposed directly to host. Mutually exclusive with normal operation.                                                          |

Button mapping:

- **Left press**: Start/stop recording, confirm GPS, skip GPS wait
- **Left long-press**: Sync data to server
- **Right press**: Sleep (from IDLE), set marker (during recording), request TCP backend stop from `SERVE_TCP`
- **Right long-press**: Start TCP server

## Core 1 dispatcher

`fw_init.c` launches `core1_worker_main()` on Core 1 during boot. Core 1 no longer runs a permanent SD writer loop. Instead it sits in `CORE1_MODE_IDLE` until Core 0 requests a backend.

Dispatcher modes:

- `CORE1_MODE_IDLE`
- `CORE1_MODE_STORAGE`
- `CORE1_MODE_TCP_SERVER`
- `CORE1_MODE_STOPPING`
- `CORE1_MODE_ERROR`

Core 0 enters these backends explicitly:

- Recording requests storage mode and waits for `CORE1_DISPATCH_EVENT_STORAGE_READY` before sending `STORAGE_CMD_OPEN`.
- `SERVE_TCP` requests TCP server mode and waits for `CORE1_DISPATCH_EVENT_TCP_SERVER_READY` before presenting the server as active.

The dispatcher uses distinct FIFO message families so backend control cannot be confused with storage commands:

- `CORE1_FIFO_FAMILY_DISPATCH_CMD`
- `CORE1_FIFO_FAMILY_DISPATCH_EVENT`
- `CORE1_FIFO_FAMILY_STORAGE_CMD`
- `CORE1_FIFO_FAMILY_STORAGE_EVENT`

Live-preview bulk sample transport does not use the FIFO. The FIFO is reserved for backend control, storage commands, and acknowledgements.

## Data pipeline

There are now two cross-core data paths:

- recording to SST files through the storage backend
- live preview over TCP through shared slot pools plus the TCP backend

### FIFO control mailbox

The multicore FIFO is now a control mailbox. The first FIFO word encodes a message family and command ID, so dispatcher traffic and storage traffic cannot be misinterpreted.

The FIFO carries:

- dispatcher mode requests and backend-ready or completion events
- recording storage commands and returned-buffer acknowledgements

The FIFO does not carry live sample payloads.

### Recording pipeline

Core 0 recording acquisition still lives in `data_acquisition.c`, and Core 1 storage still lives in `data_storage.c`. The main behavioral change is that recording explicitly enters storage mode first.

### Double-buffer scheme

Each sensor type uses two statically allocated buffers. Core 0 fills the active buffer via timer callbacks. When full, the active buffer pointer is sent to Core 1 via FIFO, and Core 0 immediately switches to the other buffer.

```
Core 0 (sampling)                         Core 1 (dispatcher + storage)
  request storage mode ──FIFO──────────>    enter storage backend
  wait for STORAGE_READY <──────────────    backend ready
  fill buffer A                            idle in storage session
  buffer A full ──FIFO─────────────────>   receive buffer A ptr
  switch to buffer B                       ack via FIFO
  fill buffer B                            write buffer A to SD
  buffer B full ──FIFO─────────────────>   receive buffer B ptr
  switch to buffer A                       ack via FIFO
  ...                                      write buffer B to SD
  FINISH ──FIFO────────────────────────>   close file, return to dispatcher IDLE
```

Storage-session FIFO protocol:

| Command                   | Arguments (pushed after command) | Description                                                                                                                                              |
| ------------------------- | -------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `STORAGE_CMD_OPEN`        | (none)                           | Open a new SST file. Core 1 replies with `STORAGE_EVENT_OPEN_RESULT`, then the file index and the writeable acquisition buffers.                         |
| `STORAGE_CMD_DUMP_TRAVEL` | `size`, `buffer_ptr`             | Write a travel chunk. Core 1 returns `STORAGE_EVENT_BUFFER_RETURNED` and the buffer pointer immediately so Core 0 can keep sampling.                     |
| `STORAGE_CMD_DUMP_IMU`    | `size`, `buffer_ptr`             | Write an IMU chunk. Same acknowledge-and-continue pattern.                                                                                               |
| `STORAGE_CMD_DUMP_GPS`    | `size`, `buffer_ptr`             | Write a GPS chunk. Same acknowledge-and-continue pattern.                                                                                                |
| `STORAGE_CMD_MARKER`      | (none)                           | Write a zero-length marker chunk to the file after Core 0 has flushed any in-flight travel/IMU data.                                                     |
| `STORAGE_CMD_FINISH`      | (none)                           | Close the current SST file. Remaining travel/IMU/GPS data must already have been flushed by Core 0. The storage backend then returns to dispatcher idle. |

### Live preview pipeline

Live preview is available only while the firmware is in `SERVE_TCP`. Recording and live preview are mutually exclusive in the current implementation.

The lifecycle is:

1. Core 0 connects WiFi and requests `CORE1_MODE_TCP_SERVER`.
2. Core 1 brings up the TCP backend and accepts a single client.
3. The client either speaks the file protocol or sends a live protocol frame with `LIVE_PROTOCOL_MAGIC`.
4. For `LIVE_FRAME_START_LIVE`, Core 1 writes a shared `live_start_request` and marks the shared control state as start-requested.
5. Core 0 services that request from the `SERVE_TCP` loop, refreshes stored calibration for the active travel and IMU sensors, negotiates the requested rates, powers or configures GPS if needed, and arms preview timers.
6. Core 0 batches travel, IMU, and GPS records into shared slot pools and marks completed slots `READY`.
7. Core 1 drains `READY` slots, wraps them in live frames, and sends them over TCP.
8. `STOP_LIVE`, disconnects, or user stop requests drive the same stop handshake so Core 0 tears down acquisition before Core 1 accepts another live client.

Live preview uses fixed shared slot pools in `live_stream_shared`:

- travel slots: 6
- IMU slots: 6
- GPS slots: 4

Each slot has explicit state and ownership:

- `FREE`
- `FILLING`
- `READY`
- `SENDING`

Core 0 owns `FREE -> FILLING -> READY`. Core 1 owns `READY -> SENDING -> FREE`. Memory barriers are used around publish and release transitions. If Core 0 cannot find a free slot for a stream, it overwrites the oldest `READY` slot for that stream and increments that stream's dropped-batch counter.

### Sample rates and buffer sizes

Recording buffers:

- **Travel**: 1000 Hz, buffer 2048 records (4 bytes each = 8 KB)
- **IMU**: 1000 Hz, buffer scales with IMU count: `(IMU_COUNT + 1) * 512` records (12 bytes each)
- **GPS**: Fix interval 300ms, poll interval 200ms, buffer 30 records (46 bytes each). GPS data flows through a callback (`on_gps_fix`) rather than a dedicated timer-fill pattern; the callback writes directly to the GPS buffer during `RECORD` state.

Live preview defaults and capacities:

- **Publish cadence**: 20 ms
- **Travel max rate**: 1000 Hz, 32 records per slot
- **IMU max rate**: 1000 Hz, 96 IMU records per slot
- **GPS max fix rate**: 10 Hz, 16 records per slot

Travel and IMU rates are negotiated per session and implemented with repeating timers on Core 0. GPS remains event-driven: the UART drain timer continues to call `gps.process()`, and completed fixes are routed through `gps_fix_router_on_fix()` either into recording or into the live GPS slot pool.

## SST binary file format (v4)

The SST format is a Type-Length-Value encoded binary format. **This firmware is the source of truth for the format specification.** All structs are packed and little-endian.

### File header (16 bytes)

| Offset | Size | Field       | Description                           |
| ------ | ---- | ----------- | ------------------------------------- |
| 0      | 3    | `magic`     | Always `"SST"`                        |
| 3      | 1    | `version`   | Currently `4`                         |
| 4      | 4    | `padding`   | Reserved, zero                        |
| 8      | 8    | `timestamp` | `time_t` UTC epoch of recording start |

### Chunk header (3 bytes, packed)

| Offset | Size | Field    | Description                                         |
| ------ | ---- | -------- | --------------------------------------------------- |
| 0      | 1    | `type`   | `CHUNK_TYPE_*` identifier                           |
| 1      | 2    | `length` | Payload length in bytes (not including this header) |

### Chunk types

| Type       | ID     | Record size                      | Description                                                                                                                                                                                                      |
| ---------- | ------ | -------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `RATES`    | `0x00` | 3 bytes                          | Sample rate declarations. Must immediately follow the file header. One `samplerate_record` per sensor type: `{uint8_t type, uint16_t rate_hz}`                                                                   |
| `TRAVEL`   | `0x01` | 4 bytes                          | Suspension travel data. Array of `{uint16_t fork_angle, uint16_t shock_angle}`                                                                                                                                   |
| `MARKER`   | `0x02` | 0 bytes                          | User-placed marker (zero-length payload). Splits recording for segment analysis.                                                                                                                                 |
| `IMU`      | `0x03` | 12 bytes                         | 6-axis IMU data. Array of `{int16_t ax, ay, az, gx, gy, gz}`. Records interleave all active IMUs in meta order (frame, fork, rear).                                                                              |
| `IMU_META` | `0x04` | 9 bytes per entry + 1 byte count | IMU metadata. Precedes all IMU data chunks. First byte is active IMU count, followed by that many `{uint8_t location_id, float accel_lsb_per_g, float gyro_lsb_per_dps}`. Location IDs: 0=Frame, 1=Fork, 2=Rear. |
| `GPS`      | `0x05` | 46 bytes                         | GPS fix data. `{uint32_t date, uint32_t time_ms, double lat, double lon, float alt, float speed, float heading, uint8_t fix_mode, uint8_t satellites, float epe_2d, float epe_3d}`                               |

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

| Implementation       | File           | Hardware                       | Comm           |
| -------------------- | -------------- | ------------------------------ | -------------- |
| AS5600 rotational    | `rotational.c` | AS5600 magnetic rotary encoder | I2C (hardware) |
| Linear potentiometer | `linear.c`     | ADC-based linear sensor        | ADC            |

The global `fork_sensor` and `shock_sensor` instances are defined in the matching implementation file, selected by `FORK_LINEAR` / `SHOCK_LINEAR` compile flags.

### IMU sensors (`src/sensor/imu/`)

Interface: `struct imu_sensor` with `init`, `check_availability`, `read_raw`, `read_temperature`, `temperature_celsius`.

| Implementation | File        | Comm       |
| -------------- | ----------- | ---------- |
| MPU6050        | `mpu6050.c` | I2C only   |
| LSM6DSO        | `lsm6dso.c` | I2C or SPI |

Up to 3 IMU positions: **Frame** (on bike frame), **Fork** (on fork lowers), **Rear** (on rear triangle). Each position's chip and protocol are selected independently via cmake variables.

The `imu_sensor.c` layer adds:

- **Two-phase calibration**: stationary (gyro bias, gravity vector) then tilted (forward direction from gravity shift). Builds a 3x3 rotation matrix mapping sensor frame to bike frame (X=forward, Y=left, Z=up).
- **Temperature compensation**: Applies drift correction based on temperature difference from calibration point.
- **Coordinate transform**: All readings are rotated to bike frame before recording.

### GPS sensor (`src/sensor/gps/`)

Interface: `struct gps_sensor` with `init`, `configure`, `process`, `send_command`, `hot_start`, `cold_start`, `power_on`, `power_off`.

| Implementation | File      | Comm                             |
| -------------- | --------- | -------------------------------- |
| Quectel LC76G  | `lc76g.c` | UART with IRQ-driven ring buffer |

Uses a **forked lwgps** library configured to parse Quectel proprietary PQTM messages (PVT + EPE) instead of standard NMEA. Standard NMEA output is disabled during configuration to reduce UART traffic.

The `gps_sensor.c` layer implements a **fix quality tracker**: requires 10 consecutive 3D fixes with >=6 satellites and EPE<=6m. Bad fixes roll back the counter by 3 (hysteresis to avoid oscillation).

GPS is power-managed: powered off after initial configuration, powered on when entering `GPS_WAIT` or when a live preview session needs GPS, and powered off again when recording or live preview stops.

## Calibration system (`src/fw/calibration_*.c`)

Calibration data is stored in a `CALIBRATION` file on the SD card using a simple magic-byte-delimited binary format.

Stored calibration is reapplied both when starting a recording session and when starting a live preview session, so travel baseline or IMU orientation can be refreshed after sensor reconnects without rebooting.

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

`sync_recorded_data()` in `src/fw/data_sync.c` connects to a configured server (`sst_server:sst_server_port`), pushes each SST file with a header containing board ID + file size + filename, and moves successfully sent files to `uploaded/`.

### TCP server (remote access and live preview)

The TCP backend listens on port 1557 with mDNS service `_gosst._tcp`. Core 1 owns the server loop while Core 0 remains in the `SERVE_TCP` state handler.

The backend supports two protocol modes on the same socket:

- **File protocol**: the pre-existing integer-based directory listing, download, and trash operations.
- **Live preview protocol**: a framed binary protocol selected when the first 4 bytes on the connection match `LIVE_PROTOCOL_MAGIC`.

File-serving operations remain:

- **Directory listing** (file ID 0): Returns board ID, sample rate, and metadata (name, size, timestamp) for all SST files.
- **File download** (file ID > 0): Streams the requested SST file, then moves it to `uploaded/`.
- **File trash** (file ID < 0): Moves the file to `trash/`.

The live preview protocol uses explicit length-prefixed frames:

- client frames: `LIVE_FRAME_START_LIVE`, `LIVE_FRAME_STOP_LIVE`, `LIVE_FRAME_PING`
- server control frames: `LIVE_FRAME_START_LIVE_ACK`, `LIVE_FRAME_STOP_LIVE_ACK`, `LIVE_FRAME_ERROR`, `LIVE_FRAME_PONG`
- server metadata or data frames: `LIVE_FRAME_SESSION_HEADER`, `LIVE_FRAME_TRAVEL_BATCH`, `LIVE_FRAME_IMU_BATCH`, `LIVE_FRAME_GPS_BATCH`, `LIVE_FRAME_SESSION_STATS`

`LIVE_FRAME_SESSION_HEADER` returns the accepted rates, exact realized periods or intervals, session timestamps, active IMU mask and scale factors, queue capacities, and session flags. Batch frames carry stream-local sequence numbers, the first sample index and monotonic timestamp, sample counts, queue depth snapshots, and dropped-batch counters so the desktop can reconstruct timing without using packet arrival timestamps.

Only one client is supported at a time. Live preview teardown is serialized so a disconnected live session cannot leave shared control state behind for the next client.

### USB Mass Storage (MSC)

When USB is connected at boot, the device enters MSC mode and exposes the SD card as a USB mass storage device. Uses TinyUSB. Mutually exclusive with normal operation (MSC disables stdio USB; debug builds use `USB_UART_DEBUG` to swap MSC for USB serial).

## Hardware interfaces

### PIO I2C

A PIO-based I2C implementation (`src/pio_i2c/`) on PIO0 SM0 is used for the DS3231 RTC and I2C display. This frees up both hardware I2C peripherals for sensors.

### Pin allocation (default)

| Function                     | Pins                                 | Interface    |
| ---------------------------- | ------------------------------------ | ------------ |
| PIO I2C (RTC, display)       | GP2 (SDA), GP3 (SCL)                 | PIO0         |
| Buttons                      | GP4 (left), GP5 (right)              | GPIO IRQ     |
| Fork sensor (rotational)     | GP8 (SDA), GP9 (SCL)                 | I2C0         |
| Fork sensor (linear)         | GP26 (ADC0)                          | ADC          |
| Display (SPI variant)        | GP10-13                              | SPI1         |
| Shock sensor (rotational)    | GP14 (SDA), GP15 (SCL)               | I2C1         |
| MicroSD (SPI)                | GP16-19                              | SPI0         |
| MicroSD (SDIO)               | GP17-22                              | PIO1         |
| GPS (LC76G)                  | GP0 (TX), GP1 (RX)                   | UART0        |
| IMU (I2C, various positions) | Shares I2C0/I2C1 with travel sensors | I2C          |
| IMU (SPI variant)            | GP10-13                              | SPI1         |
| Battery voltage              | GP29 (ADC3)                          | ADC (VSYS/3) |

## Build system

### CMake presets

The firmware supports many hardware configurations via cmake cache variables. `generate_cmake_presets.py` is an interactive script that generates `CMakePresets.json` with the user's chosen configuration.

### Key cmake variables

| Variable           | Values                       | Description                                      |
| ------------------ | ---------------------------- | ------------------------------------------------ |
| `PICO_BOARD`       | `pico_w`, `pico2_w`          | Target board                                     |
| `SPI_MICROSD`      | `ON`/`OFF`                   | SPI vs SDIO for SD card                          |
| `DISP_PROTO`       | `PIO_I2C`, `SPI`             | Display connection                               |
| `FORK_LINEAR`      | `ON`/`OFF`                   | Linear (ADC) vs rotational (AS5600) fork sensor  |
| `SHOCK_LINEAR`     | `ON`/`OFF`                   | Linear (ADC) vs rotational (AS5600) shock sensor |
| `GPS_MODULE`       | `NONE`, `LC76G`              | GPS module selection                             |
| `IMU_FRAME`        | `NONE`, `MPU6050`, `LSM6DSO` | Frame IMU chip                                   |
| `IMU_FORK`         | `NONE`, `MPU6050`, `LSM6DSO` | Fork IMU chip                                    |
| `IMU_REAR`         | `NONE`, `MPU6050`, `LSM6DSO` | Rear IMU chip                                    |
| `IMU_*_PROTO`      | `I2C`, `SPI`                 | IMU communication protocol (per position)        |
| `USB_UART_DEBUG`   | `ON`/`OFF`                   | Swap MSC for USB serial debug output             |
| `CMAKE_BUILD_TYPE` | `Release`, `Debug`           | Debug enables printf logging                     |

### External dependencies

| Library                          | Path        | Purpose                                                       |
| -------------------------------- | ----------- | ------------------------------------------------------------- |
| no-OS-FatFS-SD-SDIO-SPI-RPi-Pico | `external/` | FAT filesystem + SD card driver (SPI and SDIO)                |
| pico-as5600                      | `external/` | AS5600 rotary encoder driver                                  |
| pico-ssd1306                     | `external/` | SSD1306 OLED display driver (I2C and SPI)                     |
| lwgps                            | `external/` | Lightweight GPS NMEA parser (forked for Quectel PQTM support) |
| Pico SDK 2.2.0                   | System      | RP2040/RP2350 HAL, WiFi, lwIP, TinyUSB                        |
| Pico Extras                      | System      | Sleep, AON timer                                              |

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
