# Management Protocol Specification

## Overview

The firmware exposes a framed management protocol on the existing TCP server port `1557` alongside the unchanged `LIVE` protocol. Clients must begin the connection with the management magic value so the server can select the `MGMT` protocol instead of `LIVE`.

The TCP service is advertised via mDNS as `_gosst._tcp` so clients can discover the device on the local network.

The server accepts one client at a time. Management traffic is request-ID based, little-endian, and length-prefixed. A connected management client occupies the listener exclusively, so there is never a concurrent `LIVE` client.

## Transport Rules

- Management runs over TCP as a byte stream. TCP packet boundaries are not meaningful; clients must buffer until a full header and the full payload are available.
- All numeric fields in headers and payloads are little-endian.
- The current implementation uses management protocol version 2.
- Client-to-server control and CONFIG upload payloads are limited to 512 bytes. The maximum client-to-server frame size is therefore 528 bytes: 16-byte header plus up to 512 bytes of payload.
- Server-to-client `FILE_CHUNK` payloads may be larger. Each download advertises its exact chunk limit in `FILE_BEGIN.max_chunk_payload`; clients must use that value instead of hard-coding the firmware's current tuning.
- The server processes requests serially and does not interleave response sequences. Clients should not pipeline requests. Wait for the terminal response for a request before sending the next request.
- Terminal responses are:
  - `LIST_DIR_REQ`: `LIST_DIR_DONE` or `ERROR`
  - `GET_FILE_REQ`: `FILE_END` or `ERROR`
  - `TRASH_FILE_REQ`: `ACTION_RESULT`
  - `MARK_SST_UPLOADED_REQ`: `ACTION_RESULT`
  - `PUT_FILE_BEGIN`: `ACTION_RESULT`
  - `PUT_FILE_COMMIT`: `ACTION_RESULT`
  - `SET_TIME_REQ`: `ACTION_RESULT`
  - `PING`: `PONG`
- The CONFIG upload transaction is the only multi-frame request flow. `PUT_FILE_BEGIN`, every `PUT_FILE_CHUNK`, and `PUT_FILE_COMMIT` for one upload must use the same `request_id`.

## Header

All management frames start with this packed header:

```c
struct management_frame_header {
    uint32_t magic;
    uint16_t version;
    uint16_t frame_type;
    uint32_t request_id;
    uint32_t payload_length;
} __attribute__((packed));
```

Header layout, 16 bytes total:

| Offset | Size | Type     | Field            |
| ------ | ---- | -------- | ---------------- |
| 0      | 4    | `uint32` | `magic`          |
| 4      | 2    | `uint16` | `version`        |
| 6      | 2    | `uint16` | `frame_type`     |
| 8      | 4    | `uint32` | `request_id`     |
| 12     | 4    | `uint32` | `payload_length` |

- `magic`: `0x544D474D` (`"MGMT"`, little-endian)
- `version`: `2`
- `request_id`: echoed back on all response frames for that request. For CONFIG upload, the same `request_id` must be reused on `PUT_FILE_BEGIN`, all `PUT_FILE_CHUNK` frames, and `PUT_FILE_COMMIT`.
- `payload_length`: payload bytes immediately following the header

## Frame Types

Requests:

- `LIST_DIR_REQ = 1`
- `GET_FILE_REQ = 2`
- `TRASH_FILE_REQ = 3`
- `PUT_FILE_BEGIN = 4`
- `PUT_FILE_CHUNK = 5`
- `PUT_FILE_COMMIT = 6`
- `SET_TIME_REQ = 7`
- `PING = 8`
- `MARK_SST_UPLOADED_REQ = 9`

Responses:

- `LIST_DIR_ENTRY = 16`
- `LIST_DIR_DONE = 17`
- `FILE_BEGIN = 18`
- `FILE_CHUNK = 19`
- `FILE_END = 20`
- `ACTION_RESULT = 21`
- `ERROR = 22`
- `PONG = 23`

## Logical Directories And File Classes

Directory IDs:

- `ROOT = 1`
- `UPLOADED = 2`
- `TRASH = 3`

File classes:

- `CONFIG = 1`
- `ROOT_SST = 2`
- `UPLOADED_SST = 3`
- `TRASH_SST = 4`

Operation matrix:

| Operation                     | CONFIG | ROOT_SST | UPLOADED_SST | TRASH_SST |
| ----------------------------- | ------ | -------- | ------------ | --------- |
| `LIST_DIR(ROOT)`              | Yes    | Yes      | No           | No        |
| `LIST_DIR(UPLOADED)`          | No     | No       | Yes          | No        |
| `LIST_DIR(TRASH)`             | No     | No       | No           | Yes       |
| `GET_FILE`                    | Yes    | Yes      | Yes          | Yes       |
| `TRASH_FILE`                  | No     | Yes      | No           | No        |
| `MARK_SST_UPLOADED`           | No     | Yes      | No           | No        |
| `PUT_FILE_BEGIN/CHUNK/COMMIT` | Yes    | No       | No           | No        |

Clients never send arbitrary path strings. Recordings are addressed by `record_id` and translated internally to `00000.SST`, `uploaded/00000.SST`, or `trash/00000.SST`.

## Result Codes

`ACTION_RESULT` and `ERROR` frames use these codes:

- `OK = 0`
- `INVALID_REQUEST = -1`
- `NOT_FOUND = -2`
- `BUSY = -3`
- `IO_ERROR = -4`
- `VALIDATION_ERROR = -5`
- `UNSUPPORTED_TARGET = -6`
- `INTERNAL_ERROR = -7`

`ERROR` is used for request-scoped failures on operations that do not return `ACTION_RESULT`, and for fatal protocol/session errors. Malformed framing and other unrecoverable session errors disconnect the client; normal request failures do not. `ACTION_RESULT` is used only for `TRASH_FILE_REQ`, `MARK_SST_UPLOADED_REQ`, `PUT_FILE_BEGIN`, `PUT_FILE_COMMIT`, and `SET_TIME_REQ`.

## Request Payloads

```c
struct management_list_dir_req {
    uint16_t dir_id;
    uint16_t reserved;
};

struct management_get_file_req {
    uint16_t file_class;
    uint16_t reserved;
    int32_t record_id;
};

struct management_trash_file_req {
    int32_t record_id;
};

struct management_mark_sst_uploaded_req {
    int32_t record_id;
};

struct management_put_file_begin_req {
    uint16_t file_class;
    uint16_t reserved;
    uint64_t file_size;
};

struct management_set_time_req {
    uint32_t utc_seconds;
    uint32_t micros;
};
```

Reserved fields:

- Send zero for forward compatibility.
- The current implementation ignores them on receive.

### `LIST_DIR_REQ` payload, 4 bytes

| Offset | Size | Type     | Field      |
| ------ | ---- | -------- | ---------- |
| 0      | 2    | `uint16` | `dir_id`   |
| 2      | 2    | `uint16` | `reserved` |

### `GET_FILE_REQ` payload, 8 bytes

| Offset | Size | Type     | Field        |
| ------ | ---- | -------- | ------------ |
| 0      | 2    | `uint16` | `file_class` |
| 2      | 2    | `uint16` | `reserved`   |
| 4      | 4    | `int32`  | `record_id`  |

- For `file_class = CONFIG`, `record_id` is ignored. Clients should send `0`.
- For SST file classes, valid `record_id` range is `0..99999`.

### `TRASH_FILE_REQ` payload, 4 bytes

| Offset | Size | Type    | Field       |
| ------ | ---- | ------- | ----------- |
| 0      | 4    | `int32` | `record_id` |

- Valid `record_id` range is `0..99999`.

### `MARK_SST_UPLOADED_REQ` payload, 4 bytes

| Offset | Size | Type    | Field       |
| ------ | ---- | ------- | ----------- |
| 0      | 4    | `int32` | `record_id` |

- Valid `record_id` range is `0..99999`.

### `PUT_FILE_BEGIN` payload, 12 bytes

| Offset | Size | Type     | Field        |
| ------ | ---- | -------- | ------------ |
| 0      | 2    | `uint16` | `file_class` |
| 2      | 2    | `uint16` | `reserved`   |
| 4      | 8    | `uint64` | `file_size`  |

- `file_size` is the exact total number of raw file bytes that must later be sent across all `PUT_FILE_CHUNK` frames for this upload.
- Only `file_class = CONFIG` is currently supported.

### `PUT_FILE_CHUNK` payload, `0..512` bytes

- Payload is raw file bytes only. There is no inner structure.
- The current implementation accepts chunk payloads up to 512 bytes.

### `PUT_FILE_COMMIT` payload, 0 bytes

- No payload.

### `SET_TIME_REQ` payload, 8 bytes

| Offset | Size | Type     | Field         |
| ------ | ---- | -------- | ------------- |
| 0      | 4    | `uint32` | `utc_seconds` |
| 4      | 4    | `uint32` | `micros`      |

- `utc_seconds` is a Unix UTC timestamp in whole seconds.
- `micros` is the microsecond offset within that second.

### `PING` payload, 0 bytes

- No payload.

## Response Payloads

```c
struct management_list_dir_entry_frame {
    uint16_t dir_id;
    uint16_t file_class;
    int32_t record_id;
    uint64_t file_size;
    int64_t timestamp_utc;
    uint32_t duration_ms;
    uint8_t sst_version;
    char name[12];
};

struct management_list_dir_done_frame {
    uint32_t entry_count;
};

struct management_file_begin_frame {
    uint16_t file_class;
    uint16_t reserved;
    int32_t record_id;
    uint64_t file_size;
    uint32_t max_chunk_payload;
    char name[12];
};

struct management_action_result_frame {
    int32_t result_code;
};

struct management_error_frame {
    int32_t error_code;
};
```

### `LIST_DIR_ENTRY` payload, 41 bytes

| Offset | Size | Type       | Field           |
| ------ | ---- | ---------- | --------------- |
| 0      | 2    | `uint16`   | `dir_id`        |
| 2      | 2    | `uint16`   | `file_class`    |
| 4      | 4    | `int32`    | `record_id`     |
| 8      | 8    | `uint64`   | `file_size`     |
| 16     | 8    | `int64`    | `timestamp_utc` |
| 24     | 4    | `uint32`   | `duration_ms`   |
| 28     | 1    | `uint8`    | `sst_version`   |
| 29     | 12   | `char[12]` | `name`          |

### `LIST_DIR_DONE` payload, 4 bytes

| Offset | Size | Type     | Field         |
| ------ | ---- | -------- | ------------- |
| 0      | 4    | `uint32` | `entry_count` |

- `entry_count` is the number of `LIST_DIR_ENTRY` frames emitted for that request.

### `FILE_BEGIN` payload, 32 bytes

| Offset | Size | Type       | Field        |
| ------ | ---- | ---------- | ------------ |
| 0      | 2    | `uint16`   | `file_class` |
| 2      | 2    | `uint16`   | `reserved`   |
| 4      | 4    | `int32`    | `record_id`  |
| 8      | 8    | `uint64`   | `file_size`  |
| 16     | 4    | `uint32`   | `max_chunk_payload` |
| 20     | 12   | `char[12]` | `name`       |

- For `CONFIG`, `record_id` is `0`.
- `file_size` is the authoritative byte count for the file transfer that follows.
- `max_chunk_payload` is the largest `FILE_CHUNK` payload the server will emit for this transfer.

### `FILE_CHUNK` payload, `0..FILE_BEGIN.max_chunk_payload` bytes

- Payload is raw file bytes only. There is no inner structure.
- The current implementation emits chunks up to `FILE_BEGIN.max_chunk_payload` bytes.
- Clients must parse by frame length and must not assume a socket read contains exactly one complete frame.

### `FILE_END` payload, 0 bytes

- No payload.

### `ACTION_RESULT` payload, 4 bytes

| Offset | Size | Type    | Field         |
| ------ | ---- | ------- | ------------- |
| 0      | 4    | `int32` | `result_code` |

### `ERROR` payload, 4 bytes

| Offset | Size | Type    | Field        |
| ------ | ---- | ------- | ------------ |
| 0      | 4    | `int32` | `error_code` |

### `PONG` payload, 0 bytes

- No payload.

Fixed-size `name[12]` fields:

- `name` is a fixed 12-byte byte array.
- Names are ASCII and zero-padded.
- `CONFIG` is encoded as `CONFIG` followed by zero padding.
- SST names are encoded as `xxxxx.SST` followed by zero padding.
- Clients should treat bytes up to the first zero byte as the displayed filename.

## Response Rules

- `LIST_DIR_REQ` returns zero or more `LIST_DIR_ENTRY` frames followed by `LIST_DIR_DONE`.
- `LIST_DIR_REQ` error matrix:
  - `ERROR(BUSY)` if a CONFIG upload is already open.
  - `ERROR(INVALID_REQUEST)` if `dir_id` is not one of `ROOT`, `UPLOADED`, or `TRASH`.
- `GET_FILE_REQ` returns `FILE_BEGIN`, zero or more `FILE_CHUNK` frames, then `FILE_END`.
- `GET_FILE_REQ` may return a single `ERROR` frame instead of the normal response sequence when the request is rejected.
- `GET_FILE_REQ` error matrix:
  - `ERROR(BUSY)` if a CONFIG upload is already open.
  - `ERROR(UNSUPPORTED_TARGET)` if `file_class` is not one of `CONFIG`, `ROOT_SST`, `UPLOADED_SST`, or `TRASH_SST`.
  - `ERROR(INVALID_REQUEST)` if an SST `record_id` is outside `0..99999`.
  - `ERROR(NOT_FOUND)` if the requested file does not exist.
  - `ERROR(IO_ERROR)` if the file cannot be opened.
- `GET_FILE_REQ` is read-only; the server never renames or moves a file as a side effect of a successful download. `uploaded/` transitions happen only via `MARK_SST_UPLOADED_REQ`, which the client issues after it has validated the downloaded bytes.
- `TRASH_FILE_REQ` returns one `ACTION_RESULT`.
- `TRASH_FILE_REQ` error matrix:
  - `ACTION_RESULT(BUSY)` if a CONFIG upload is already open.
  - `ACTION_RESULT(INVALID_REQUEST)` if `record_id` is outside `0..99999`.
  - `ACTION_RESULT(NOT_FOUND)` if the root SST file does not exist.
  - `ACTION_RESULT(IO_ERROR)` if the rename fails.
- `MARK_SST_UPLOADED_REQ` returns one `ACTION_RESULT`. Target is the root SST `xxxxx.SST`; it is moved to `uploaded/xxxxx.SST`. Error matrix: bad payload length yields `INVALID_REQUEST`; upload in progress on the same session yields `BUSY`; invalid `record_id` (negative or > 99999) yields `INVALID_REQUEST`; no file at the root path (already moved, trashed, or never existed) yields `NOT_FOUND`; rename failure yields `IO_ERROR`.
- `PUT_FILE_BEGIN` returns one `ACTION_RESULT`.
- `PUT_FILE_BEGIN` error matrix:
  - `ACTION_RESULT(BUSY)` if a CONFIG upload is already open.
  - `ACTION_RESULT(UNSUPPORTED_TARGET)` if `file_class` is not `CONFIG`.
  - `ACTION_RESULT(IO_ERROR)` if the staging file cannot be opened.
- `PUT_FILE_CHUNK` returns no per-chunk acknowledgement on success.
- `PUT_FILE_CHUNK` may return `ERROR(IO_ERROR)` if the server cannot write the chunk to the staged file. In that case the staged upload is aborted and the client must start a new upload with `PUT_FILE_BEGIN`.
- `PUT_FILE_COMMIT` returns one `ACTION_RESULT` after the staged file has been validated, committed, and applied.
- `PUT_FILE_COMMIT` result codes in normal operation are `OK`, `INVALID_REQUEST`, `VALIDATION_ERROR`, or `IO_ERROR`.
- `SET_TIME_REQ` returns one `ACTION_RESULT`.
- `SET_TIME_REQ` result codes in normal operation are `OK`, `BUSY`, or `INTERNAL_ERROR`.
- `PING` returns `PONG`.
- Fatal protocol violations disconnect the client after sending `ERROR(INVALID_REQUEST)` if possible. Examples include wrong payload length, unknown frame type, invalid magic/version, `PUT_FILE_CHUNK` before a successful `PUT_FILE_BEGIN`, `PUT_FILE_CHUNK` with the wrong `request_id`, `PUT_FILE_CHUNK` that would exceed the announced `file_size`, and `PUT_FILE_COMMIT` with the wrong `request_id` or before a successful `PUT_FILE_BEGIN`.
- Fatal I/O failures while streaming a directory or file transfer close the connection rather than producing a recoverable per-request result.

## Listing Semantics

- `LIST_DIR(ROOT)` includes `CONFIG` if the file exists, plus root-level `?????.SST` recordings.
- `LIST_DIR(UPLOADED)` includes only `uploaded/?????.SST`.
- `LIST_DIR(TRASH)` includes only `trash/?????.SST`.
- `LIST_DIR(ROOT)` emits `CONFIG` first if it exists. SST entries follow in filesystem iteration order and are not sorted by record ID or timestamp.
- SST entries are emitted for matching `?????.SST` filenames. Metadata is read from the SST file during each listing.
- `timestamp_utc`, `duration_ms`, and `sst_version` are populated from the SST header and chunk metadata for `.SST` files. If metadata cannot be read, those fields use the zero values returned by the SST scanner.
- `CONFIG` list entries use zero for timestamp, duration, version, and record ID.

## Config Upload Flow

`CONFIG` replacement is staged and does not replace the committed `CONFIG` file until the uploaded bytes have been fully received, validated, and committed.

Upload rules:

- Only one upload may be open at a time.
- `PUT_FILE_BEGIN`, all `PUT_FILE_CHUNK` frames, and `PUT_FILE_COMMIT` for one upload must use the same `request_id`.
- `file_size` in `PUT_FILE_BEGIN` is the exact byte count expected across all `PUT_FILE_CHUNK` payloads.
- If a `PUT_FILE_CHUNK` would make the total received byte count exceed `file_size`, the server treats it as `INVALID_REQUEST` and disconnects the client.
- If `PUT_FILE_COMMIT` arrives before the total received byte count exactly matches `file_size`, the server returns `ACTION_RESULT(INVALID_REQUEST)`.
- While an upload is open:
  - `LIST_DIR_REQ` and `GET_FILE_REQ` are rejected with `ERROR(BUSY)`.
  - `TRASH_FILE_REQ`, `MARK_SST_UPLOADED_REQ`, `PUT_FILE_BEGIN`, and `SET_TIME_REQ` return `ACTION_RESULT(BUSY)`.
  - `PING` remains available.

Flow:

1. Client sends `PUT_FILE_BEGIN(CONFIG, file_size)`.
2. Server opens a staging file and returns `ACTION_RESULT`.
3. Client streams raw config bytes through `PUT_FILE_CHUNK` frames.
4. Client sends `PUT_FILE_COMMIT`.
5. The server validates the staged file into a concrete `struct config` snapshot.
6. If validation succeeds, the server replaces the committed `CONFIG` file with the validated staged file.
7. The server applies the same validated snapshot to the active runtime configuration where immediate runtime application is supported.
8. The server returns `ACTION_RESULT`.

If validation fails, the committed `CONFIG` file stays unchanged and `PUT_FILE_COMMIT` returns `VALIDATION_ERROR`.

Accepted Wi-Fi-related config changes (`WIFI_MODE`, `STA_SSID`, `STA_PSK`, `AP_SSID`, `AP_PSK`, `COUNTRY`) are persisted immediately but only take effect after restart. The current Wi-Fi mode, credentials, mDNS advertisement, and active TCP listener are not hot-reconfigured in place.

## Time Update Semantics

`SET_TIME_REQ` applies the requested UTC wall-clock update.

- `utc_seconds` is interpreted as a Unix UTC timestamp in whole seconds.
- `micros` is interpreted as the microsecond offset within that second.
- The current implementation returns `ACTION_RESULT(BUSY)` if a CONFIG upload is in progress.
- The current implementation returns `ACTION_RESULT(INTERNAL_ERROR)` if the time update cannot be applied.
- The onboard always-on timer and DS3231 are both updated.
- The active live session keeps its original `session_start_utc` value.
- Live batch timing remains monotonic because live streaming uses monotonic timestamps, not wall-clock deltas.

## Protocol Selection

The TCP server starts each connection in `UNKNOWN` mode and lets registered protocol handlers inspect the buffered initial bytes.

- `LIVE` claims the connection when the first 4 bytes match `LIVE_PROTOCOL_MAGIC`.
- `MGMT` claims the connection when the first 4 bytes match `MANAGEMENT_PROTOCOL_MAGIC`.
- Unknown initial traffic is rejected.

## Compatibility Notes

- The `LIVE` protocol wire format is unchanged by the management refactor.
- The legacy frameless server-side file protocol is no longer active.
- Management v2 is a breaking wire update. Clients must send header `version = 2`; v1 management frames are rejected as invalid version traffic.
