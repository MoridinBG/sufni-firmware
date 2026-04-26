# Live Streaming Specification

## Overview

The firmware exposes an on-demand live streaming protocol on TCP port `1557`. A client connects, selects the `LIVE` protocol by sending the live magic value first, and then negotiates a live session with independent requested rates for travel, IMU, and GPS.

The TCP service is advertised via mDNS as `_gosst._tcp` so clients can discover the device on the local network.

Current protocol constraints:

- Preview only — no simultaneous SST recording.
- The TCP server accepts one client total at a time, so there can be only one active `LIVE` session with one client.
- Rates are negotiated once at session start; no mid-stream reconfiguration.
- Calibrated travel and IMU values only.
- GPS streams all fixes using the existing `gps_record` payload.
- No live-session state persists across reconnects; each reconnect starts a new session from a clean state.

## Transport Rules

- `LIVE` runs over TCP as a byte stream. TCP packet boundaries are not meaningful; clients must buffer until a full frame header and the full payload are available.
- All numeric fields in headers and payloads are little-endian.
- `LIVE` has no request ID field. Request/response matching is stateful and based on frame type and session state, not on an application-level request identifier.
- The frame header `sequence` field in client-to-server frames is currently ignored. Clients should set it to `0`.
- The frame header `sequence` field in server-to-client frames is a connection-local transmit sequence. It starts at `1` when the connection enters `LIVE` mode and increments by `1` for every outbound `LIVE` frame, including `ERROR`, `PONG`, `IDENTIFY_ACK`, `START_LIVE_ACK`, `SESSION_HEADER`, `SESSION_STATS`, batch frames, and `STOP_LIVE_ACK`.
- The current implementation rejects client frames whose `payload_length` exceeds `512` bytes. Such frames close the connection.
- Client requests may be sent while a session is active. `PING` and `IDENTIFY` remain available during startup, active streaming, and stop processing.

## Protocol Constants

### Magic & Version

- Magic: `0x4556494C` (ASCII `"LIVE"`, little-endian) — uint32
- Protocol version: `1` — uint16

### Frame Types

`enum live_frame_type`:

| Name           | Value | Direction |
| -------------- | ----- | --------- |
| START_LIVE     | 1     | request   |
| STOP_LIVE      | 2     | request   |
| PING           | 3     | request   |
| IDENTIFY       | 4     | request   |
| START_LIVE_ACK | 16    | response  |
| STOP_LIVE_ACK  | 17    | response  |
| ERROR          | 18    | response  |
| PONG           | 19    | response  |
| SESSION_HEADER | 20    | info      |
| IDENTIFY_ACK   | 21    | response  |
| TRAVEL_BATCH   | 32    | data      |
| IMU_BATCH      | 33    | data      |
| GPS_BATCH      | 34    | data      |
| SESSION_STATS  | 48    | info      |

### Stream Types

`enum live_stream_type`:

| Name   | Value |
| ------ | ----- |
| TRAVEL | 1     |
| IMU    | 2     |
| GPS    | 3     |

### Sensor Mask (bitfield)

| Sensor | Bit  |
| ------ | ---- |
| TRAVEL | 0x01 |
| IMU    | 0x02 |
| GPS    | 0x04 |

### Session Flags (bitfield)

| Flag                              | Bit  |
| --------------------------------- | ---- |
| CALIBRATED_ONLY                   | 0x01 |
| MUTUALLY_EXCLUSIVE_WITH_RECORDING | 0x02 |

### Error Codes

`enum live_start_result`:

| Name            | Value |
| --------------- | ----- |
| OK              | 0     |
| INVALID_REQUEST | -1    |
| BUSY            | -2    |
| UNAVAILABLE     | -3    |
| INTERNAL_ERROR  | -4    |

## Protocol Discrimination

The TCP server accepts a single client. On connection, the first bytes determine which protocol is active:

- Management protocol: a management frame header starting with magic `0x544D474D`.
- Live protocol: a live frame header starting with magic `0x4556494C`.

The server inspects the initial bytes and routes the connection to the appropriate protocol handler. Unknown initial traffic is rejected.

Because the listener is single-client, a connected management client excludes live streaming for the duration of that connection, and vice versa.

## Live Frame Format

All live traffic uses explicit length-prefixed framing over TCP. Each frame is a fixed-size header followed by exactly `payload_length` bytes of payload.

TCP read boundaries are not meaningful. The receiver must accumulate bytes until a complete header and payload are available before parsing.

### Frame Header (16 bytes)

`struct live_frame_header`, all fields little-endian, packed:

| Offset | Size | Type   | Field          |
| ------ | ---- | ------ | -------------- |
| 0      | 4    | uint32 | magic          |
| 4      | 2    | uint16 | version        |
| 6      | 2    | uint16 | frame_type     |
| 8      | 4    | uint32 | payload_length |
| 12     | 4    | uint32 | sequence       |

- `magic`: always `0x4556494C`.
- `version`: protocol version, currently `1`.
- `frame_type`: see Frame Types table.
- `payload_length`: byte count of the payload following this header. May be 0 (e.g. STOP_LIVE request, PONG).
- `sequence`: connection-local transmit sequence on server-to-client frames. Client-to-server values are currently ignored.

Malformed frame handling:

- If `magic != 0x4556494C`, `version != 1`, or `payload_length > 512` on a client-to-server frame, the current implementation closes the connection without sending a protocol error frame.
- If the frame header is valid but the payload size is wrong for a known frame type, the server sends `ERROR(INVALID_REQUEST)` and keeps the connection open.

## Session Negotiation

### START_LIVE Request

Frame type `1`. Payload is `struct live_start_request_frame` (16 bytes):

| Offset | Size | Type   | Field       |
| ------ | ---- | ------ | ----------- |
| 0      | 4    | uint32 | sensor_mask |
| 4      | 4    | uint32 | travel_hz   |
| 8      | 4    | uint32 | imu_hz      |
| 12     | 4    | uint32 | gps_fix_hz  |

- `sensor_mask`: bitfield selecting requested streams (see Sensor Mask table).
- `travel_hz`, `imu_hz`, `gps_fix_hz`: requested sample rates. `0` means use firmware default.
- Current implementation limits requested rates to these maxima before quantization:
  - travel: `1000` Hz
  - IMU: `1000` Hz
  - GPS: `10` Hz

`START_LIVE` response behavior:

- If no session is active and the server accepts the request for startup, the response path is `START_LIVE_ACK` followed, on success only, by `SESSION_HEADER` and an initial `SESSION_STATS`.
- If a live session is already active or a start/stop transition is already in progress, the current implementation responds with `ERROR(BUSY)` instead of `START_LIVE_ACK`.
- If the payload size is wrong, the server responds with `ERROR(INVALID_REQUEST)`.

`START_LIVE_ACK.result` meanings in the current implementation:

- `OK`: session started successfully.
- `INVALID_REQUEST`: `sensor_mask` was zero.
- `UNAVAILABLE`: the requested stream had no available sensors.
- `INTERNAL_ERROR`: calibration refresh failed, GPS setup failed, or a timer could not be started.

### STOP_LIVE Request

Frame type `2`. No payload (`payload_length = 0`).

`STOP_LIVE` behavior:

- If a session is active, the server begins stop processing and later replies with `STOP_LIVE_ACK`.
- If `STOP_LIVE` arrives during the startup handshake, the stop is queued and `STOP_LIVE_ACK` is sent after startup resolves.
- If no session is active, the current implementation still replies with `STOP_LIVE_ACK`. In that case, the `session_id` field is not meaningful.
- The current implementation does not guarantee delivery of already-produced but unsent batch frames before `STOP_LIVE_ACK`.
- If the payload size is wrong, the server responds with `ERROR(INVALID_REQUEST)`.

### PING

Frame type `3`. No payload. Server responds with `PONG` (frame type `19`, no payload).

- `PING` is available at any time while the connection is in `LIVE` mode.
- If the payload size is wrong, the server responds with `ERROR(INVALID_REQUEST)`.

### IDENTIFY

Frame type `4`. No payload (`payload_length = 0`). Requests the board's unique identity.

May be sent at any time — before, during, or between sessions. Does not start, stop, or disturb any active session. Does not require a prior START_LIVE.

- If the payload size is wrong, the server responds with `ERROR(INVALID_REQUEST)`.

### IDENTIFY_ACK Response

Frame type `21`. Payload is `struct live_identify_ack_frame` (8 bytes):

| Offset | Size | Type     | Field    |
| ------ | ---- | -------- | -------- |
| 0      | 8    | uint8[8] | board_id |

- `board_id`: the Pico's 8-byte unique board serial.

Sent immediately after receiving `IDENTIFY`. Uses the normal outbound `sequence` stream for the connection.

### START_LIVE_ACK Response

Frame type `16`. Payload is `struct live_start_ack_frame` (12 bytes):

| Offset | Size | Type   | Field                |
| ------ | ---- | ------ | -------------------- |
| 0      | 4    | int32  | result               |
| 4      | 4    | uint32 | session_id           |
| 8      | 4    | uint32 | selected_sensor_mask |

- `result`: see Error Codes table. `0` = success; negative = failure.
- `session_id`: unique session identifier (only meaningful when `result == 0`).
- `selected_sensor_mask`: bitfield of sensors actually enabled.

On success (`result == 0`), the server immediately sends SESSION_HEADER and SESSION_STATS frames following the ACK.

On failure, `session_id` and `selected_sensor_mask` are zero. No SESSION_HEADER follows.

If the TCP connection drops or the session is stopped, all live-session state is discarded. A later reconnect must negotiate a fresh session; no subscriptions, sequence continuity, or negotiated state persists.

### SESSION_HEADER

Frame type `20`. Sent once immediately after a successful START_LIVE_ACK. Payload is `struct live_session_header_frame` (64 bytes):

| Offset | Size | Type     | Field                      |
| ------ | ---- | -------- | -------------------------- |
| 0      | 4    | uint32   | session_id                 |
| 4      | 4    | uint32   | accepted_travel_hz         |
| 8      | 4    | uint32   | accepted_imu_hz            |
| 12     | 4    | uint32   | accepted_gps_fix_hz        |
| 16     | 8    | int64    | session_start_utc          |
| 24     | 8    | uint64   | session_start_monotonic_us |
| 32     | 4    | uint32   | active_imu_mask            |
| 36     | 12   | float[3] | imu_accel_lsb_per_g        |
| 48     | 12   | float[3] | imu_gyro_lsb_per_dps       |
| 60     | 4    | uint32   | flags                      |

Notes:

- The `accepted_*_hz` values are already consistent with the firmware's internal timer periods. The client derives the exact period as `1000000 / accepted_travel_hz` (microseconds) or `1000 / accepted_gps_fix_hz` (milliseconds).
- `active_imu_mask`: bitfield — bit 0 = frame, bit 1 = fork, bit 2 = rear. IMU count = `popcount(active_imu_mask)`.
- `selected_sensor_mask` is in START_LIVE_ACK and not repeated here.
- `imu_accel_lsb_per_g[3]` and `imu_gyro_lsb_per_dps[3]`: per-IMU calibration scales indexed by location (0=frame, 1=fork, 2=rear). Zero for absent IMUs.
- `flags`: see Session Flags table. The current implementation always sets both defined flags.

### STOP_LIVE_ACK Response

Frame type `17`. Payload is `struct live_stop_ack_frame` (4 bytes):

| Offset | Size | Type   | Field      |
| ------ | ---- | ------ | ---------- |
| 0      | 4    | uint32 | session_id |

### ERROR Response

Frame type `18`. Payload is `struct live_error_frame` (4 bytes):

| Offset | Size | Type  | Field      |
| ------ | ---- | ----- | ---------- |
| 0      | 4    | int32 | error_code |

See Error Codes table for values.

The current implementation uses `ERROR` for immediate request-level failures such as:

- `START_LIVE` while a live session or start/stop transition is already active: `ERROR(BUSY)`
- `START_LIVE`, `STOP_LIVE`, `PING`, or `IDENTIFY` with the wrong payload size: `ERROR(INVALID_REQUEST)`
- unknown `frame_type`: `ERROR(INVALID_REQUEST)`

`ERROR` by itself does not terminate the connection.

### PONG Response

Frame type `19`. No payload (`payload_length = 0`).

## Batch Frame Format

Data frames (TRAVEL_BATCH, IMU_BATCH, GPS_BATCH) carry samples from a single stream and a single publish window.

The payload consists of a batch header followed by sample data.

### Batch Header (28 bytes)

`struct live_batch_payload`, all fields little-endian, packed:

| Offset | Size | Type   | Field              |
| ------ | ---- | ------ | ------------------ |
| 0      | 4    | uint32 | session_id         |
| 4      | 4    | uint32 | stream_sequence    |
| 8      | 8    | uint64 | first_index        |
| 16     | 8    | uint64 | first_monotonic_us |
| 24     | 4    | uint32 | sample_count       |

- `stream_sequence`: per-stream batch sequence number (starts at 0, increments per batch for that stream).
- `first_index`: global sample index of the first entry in this batch.
- `first_monotonic_us`: monotonic timestamp (microseconds) of the first sample.
- `sample_count`: number of samples (for IMU: tick count, not record count).

Ordering rules:

- `stream_sequence` is the authoritative per-stream ordering field.
- The frame header `sequence` is the authoritative connection-level transmit ordering field.
- Receive order across different stream types is not a global timestamp order. Clients must not infer cross-stream simultaneity from frame arrival order.

Stream type is implicit in the frame type (TRAVEL_BATCH=32, IMU_BATCH=33, GPS_BATCH=34).

Payload byte length is derived from `sample_count` and the known record sizes:

- Travel: `sample_count * 4`
- IMU: `sample_count * popcount(active_imu_mask) * 12`
- GPS: `sample_count * 46`

Queue depth and drop counts are reported in SESSION_STATS, not per-batch.

The frame header's `payload_length` equals `28 + derived_payload_bytes`.

### Travel Record (4 bytes)

`struct travel_record`:

| Offset | Size | Type   | Field       |
| ------ | ---- | ------ | ----------- |
| 0      | 2    | uint16 | fork_angle  |
| 2      | 2    | uint16 | shock_angle |

Batch contains `sample_count` records. `payload_byte_length = sample_count * 4`.

### IMU Record (12 bytes)

`struct imu_record`:

| Offset | Size | Type  | Field |
| ------ | ---- | ----- | ----- |
| 0      | 2    | int16 | ax    |
| 2      | 2    | int16 | ay    |
| 4      | 2    | int16 | az    |
| 6      | 2    | int16 | gx    |
| 8      | 2    | int16 | gy    |
| 10     | 2    | int16 | gz    |

When multiple IMUs are active, each tick contains `active_imu_count` consecutive records in location order (frame, fork, rear — only those present in `active_imu_mask`). `sample_count` is the tick count. Total records = `sample_count * active_imu_count`. `payload_byte_length = sample_count * active_imu_count * 12`.

### GPS Record (46 bytes)

`struct gps_record`, packed:

| Offset | Size | Type   | Field      |
| ------ | ---- | ------ | ---------- |
| 0      | 4    | uint32 | date       |
| 4      | 4    | uint32 | time_ms    |
| 8      | 8    | double | latitude   |
| 16     | 8    | double | longitude  |
| 24     | 4    | float  | altitude   |
| 28     | 4    | float  | speed      |
| 32     | 4    | float  | heading    |
| 36     | 1    | uint8  | fix_mode   |
| 37     | 1    | uint8  | satellites |
| 38     | 4    | float  | epe_2d     |
| 42     | 4    | float  | epe_3d     |

Total: 46 bytes.

- `date`: UTC date as YYYYMMDD integer.
- `time_ms`: UTC time of day in milliseconds.
- `latitude`, `longitude`: WGS84 decimal degrees.
- `altitude`: meters.
- `speed`: m/s.
- `heading`: degrees.
- `fix_mode`: 0 = none, 1 = 2D, 2 = 3D.
- `epe_2d`, `epe_3d`: estimated position error in meters.

Clients receive all fixes — including low-quality and no-fix entries — and reconstruct GPS readiness and quality state locally.

`payload_byte_length = sample_count * 46`.

## SESSION_STATS

Frame type `48`. Sent opportunistically at approximately 500 ms intervals during an active session. Payload is `struct live_session_stats_frame` (28 bytes):

| Offset | Size | Type   | Field                  |
| ------ | ---- | ------ | ---------------------- |
| 0      | 4    | uint32 | session_id             |
| 4      | 4    | uint32 | travel_queue_depth     |
| 8      | 4    | uint32 | imu_queue_depth        |
| 12     | 4    | uint32 | gps_queue_depth        |
| 16     | 4    | uint32 | travel_dropped_batches |
| 20     | 4    | uint32 | imu_dropped_batches    |
| 24     | 4    | uint32 | gps_dropped_batches    |

Accepted rates and periods are sent once in SESSION_HEADER and do not repeat here.

## Timing Model

### Travel

Reconstruct sample timestamps from the batch metadata and the derived travel period (`1000000 / accepted_travel_hz`):

```
travel_period_us = 1000000 / accepted_travel_hz
sample_time[n] = first_monotonic_us + n * travel_period_us
```

Packet arrival time must not be used for sample timing.

### IMU

Reconstruct tick timestamps from the batch metadata and the derived IMU period (`1000000 / accepted_imu_hz`):

```
imu_period_us = 1000000 / accepted_imu_hz
tick_time[n] = first_monotonic_us + n * imu_period_us
```

Each tick contains `active_imu_count` records in the order specified by `active_imu_mask` in the session header. The batch `sample_count` is the tick count.

### GPS

GPS is event-driven. Each `gps_record` carries its own UTC fix time. A monotonic enqueue anchor is provided in the batch header for ordering relative to other streams, but fix timing comes from the GPS record itself.

### Cross-Stream

Travel and IMU have stable per-stream timing but are not exactly simultaneous. The protocol does not claim exact cross-stream synchronization.

## Backpressure

When the network cannot keep up:

- Older unsent batch frames may be discarded on a per-stream basis in favor of newer data.
- The stream's cumulative drop counter increments.
- Current queue depth and drop counts are reported in SESSION_STATS.

The specific internal buffering strategy is not part of the wire contract. Clients should surface drop counts and queue depth as indicators of link quality.

## Session Lifecycle

1. Client connects to TCP port `1557`.
2. Client sends a `LIVE` frame first so the server selects the live protocol.
3. Client sends `START_LIVE` with the requested sensor mask and requested rates.
4. The server either:
   - responds with `ERROR(BUSY)` if a live session or start/stop transition is already in progress, or
   - attempts startup and responds with `START_LIVE_ACK`.
5. On successful startup, the server sends `SESSION_HEADER`, then an initial `SESSION_STATS`, then begins streaming `TRAVEL_BATCH`, `IMU_BATCH`, and `GPS_BATCH` frames as data becomes available.
6. During the session, the client may send `PING`, `IDENTIFY`, or `STOP_LIVE` at any time.
7. The server emits `SESSION_STATS` periodically while the session remains active.
8. When the client sends `STOP_LIVE`, the server stops the session and replies with `STOP_LIVE_ACK`. Unsent batch frames may be dropped during this transition.
9. If the TCP connection closes, all live-session state for that connection is discarded. A later reconnect must negotiate a fresh session.

## Mutual Exclusion

The current firmware does not offer live preview and SST recording at the same time. Live streaming is only available in TCP service modes that enable live preview, and an active live session is mutually exclusive with recording.
