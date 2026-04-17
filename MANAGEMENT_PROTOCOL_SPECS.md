# Management Protocol Specification

## Overview

The firmware exposes a framed management protocol on the existing TCP server port `1557` alongside the unchanged `LIVE` protocol. Clients must begin the connection with the management magic value so the server can select the `MGMT` protocol instead of `LIVE`.

The server accepts one client at a time. Management traffic is request-ID based, little-endian, and length-prefixed.

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

- `magic`: `0x544D474D` (`"MGMT"`, little-endian)
- `version`: `1`
- `request_id`: echoed back on all response frames for that request
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

| Operation | CONFIG | ROOT_SST | UPLOADED_SST | TRASH_SST |
| --- | --- | --- | --- | --- |
| `LIST_DIR(ROOT)` | Yes | Yes | No | No |
| `LIST_DIR(UPLOADED)` | No | No | Yes | No |
| `LIST_DIR(TRASH)` | No | No | No | Yes |
| `GET_FILE` | Yes | Yes | Yes | Yes |
| `TRASH_FILE` | No | Yes | No | No |
| `MARK_SST_UPLOADED` | No | Yes | No | No |
| `PUT_FILE_BEGIN/CHUNK/COMMIT` | Yes | No | No | No |

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

`PUT_FILE_CHUNK` carries raw file bytes only and is limited to 512 payload bytes per frame. `PUT_FILE_COMMIT` and `PING` have no payload.

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
    char name[12];
};

struct management_action_result_frame {
    int32_t result_code;
};

struct management_error_frame {
    int32_t error_code;
};
```

`FILE_CHUNK` carries raw file bytes only. `FILE_END` and `PONG` have no payload.

## Response Rules

- `LIST_DIR_REQ` returns zero or more `LIST_DIR_ENTRY` frames followed by `LIST_DIR_DONE`.
- `GET_FILE_REQ` returns `FILE_BEGIN`, zero or more `FILE_CHUNK` frames, then `FILE_END`.
- `LIST_DIR_REQ` and `GET_FILE_REQ` may return a single `ERROR` frame instead of their normal response sequence when the request is rejected.
- `GET_FILE_REQ` is read-only; the server never renames or moves a file as a side effect of a successful download. `uploaded/` transitions happen only via `MARK_SST_UPLOADED_REQ`, which the client issues after it has validated the downloaded bytes.
- `TRASH_FILE_REQ` returns one `ACTION_RESULT`.
- `MARK_SST_UPLOADED_REQ` returns one `ACTION_RESULT`. Target is the root SST `xxxxx.SST`; it is moved to `uploaded/xxxxx.SST`. Error matrix: bad payload length yields `INVALID_REQUEST`; upload in progress on the same session yields `BUSY`; invalid `record_id` (negative or > 99999) yields `INVALID_REQUEST`; no file at the root path (already moved, trashed, or never existed) yields `NOT_FOUND`; rename failure yields `IO_ERROR`.
- `PUT_FILE_BEGIN` returns one `ACTION_RESULT`.
- `PUT_FILE_CHUNK` returns no per-chunk acknowledgement.
- `PUT_FILE_COMMIT` returns one `ACTION_RESULT` after disk commit and runtime publication complete.
- `SET_TIME_REQ` returns one `ACTION_RESULT`.
- `PING` returns `PONG`.

## Listing Semantics

- `LIST_DIR(ROOT)` includes `CONFIG` if the file exists, plus root-level `?????.SST` recordings.
- `LIST_DIR(UPLOADED)` includes only `uploaded/?????.SST`.
- `LIST_DIR(TRASH)` includes only `trash/?????.SST`.
- `timestamp_utc`, `duration_ms`, and `sst_version` are populated from the SST header/metadata for `.SST` files.
- `CONFIG` list entries use zero for timestamp, duration, version, and record ID.

## Config Upload Flow

`CONFIG` replacement is staged through `CONFIG.TMP` and crosses cores only after the file has been validated and committed on disk.

Flow:

1. Client sends `PUT_FILE_BEGIN(CONFIG, file_size)`.
2. Server opens `CONFIG.TMP` and returns `ACTION_RESULT`.
3. Client streams raw config bytes through `PUT_FILE_CHUNK` frames.
4. Client sends `PUT_FILE_COMMIT`.
5. Core 1 validates `CONFIG.TMP` into a concrete `struct config` snapshot.
6. Core 1 replaces the committed `CONFIG` file with the validated staged file.
7. Core 1 publishes the same validated snapshot to Core 0 through `management_shared`.
8. Core 0 applies the snapshot to the authoritative runtime `config` object.
9. Server returns `ACTION_RESULT` using the Core 0 completion result.

If validation fails, the committed `CONFIG` file stays unchanged and `PUT_FILE_COMMIT` returns `VALIDATION_ERROR`.

## Time Update Semantics

`SET_TIME_REQ` is forwarded from Core 1 to Core 0 through `management_shared`.

- Core 0 applies the wall-clock update through `set_system_time_utc()`.
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