#ifndef MANAGEMENT_PROTOCOL_H
#define MANAGEMENT_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

#include "ff.h"

#include "../util/config.h"

#define MANAGEMENT_PROTOCOL_MAGIC   0x544D474Du
#define MANAGEMENT_PROTOCOL_VERSION 2u

#define MANAGEMENT_PROTOCOL_CONFIG_NAME                   "CONFIG"
#define MANAGEMENT_PROTOCOL_CONFIG_STAGING_PATH           "CONFIG.TMP"
#define MANAGEMENT_PROTOCOL_NAME_LENGTH                   12u
#define MANAGEMENT_PROTOCOL_FRAME_HEADER_SIZE             16u
#define MANAGEMENT_PROTOCOL_CONTROL_MAX_PAYLOAD_SIZE      512u
#define MANAGEMENT_PROTOCOL_MAX_UPLOAD_CHUNK_PAYLOAD_SIZE MANAGEMENT_PROTOCOL_CONTROL_MAX_PAYLOAD_SIZE
#define MGMT_V2_MAX_FILE_CHUNK_PAYLOAD                    5824u
#define MGMT_V2_TX_FRAME_BUFFER_SIZE       (MANAGEMENT_PROTOCOL_FRAME_HEADER_SIZE + MGMT_V2_MAX_FILE_CHUNK_PAYLOAD)
#define MANAGEMENT_PROTOCOL_IO_BUFFER_SIZE MGMT_V2_MAX_FILE_CHUNK_PAYLOAD
#define MANAGEMENT_PROTOCOL_MAX_RX_FRAME_SIZE                                                                          \
    (MANAGEMENT_PROTOCOL_FRAME_HEADER_SIZE + MANAGEMENT_PROTOCOL_CONTROL_MAX_PAYLOAD_SIZE)

struct tcpserver;
struct tcpserver_protocol_ops;

enum management_frame_type {
    MGMT_FRAME_LIST_DIR_REQ = 1,
    MGMT_FRAME_GET_FILE_REQ = 2,
    MGMT_FRAME_TRASH_FILE_REQ = 3,
    MGMT_FRAME_PUT_FILE_BEGIN = 4,
    MGMT_FRAME_PUT_FILE_CHUNK = 5,
    MGMT_FRAME_PUT_FILE_COMMIT = 6,
    MGMT_FRAME_SET_TIME_REQ = 7,
    MGMT_FRAME_PING = 8,
    MGMT_FRAME_MARK_SST_UPLOADED_REQ = 9,

    MGMT_FRAME_LIST_DIR_ENTRY = 16,
    MGMT_FRAME_LIST_DIR_DONE = 17,
    MGMT_FRAME_FILE_BEGIN = 18,
    MGMT_FRAME_FILE_CHUNK = 19,
    MGMT_FRAME_FILE_END = 20,
    MGMT_FRAME_ACTION_RESULT = 21,
    MGMT_FRAME_ERROR = 22,
    MGMT_FRAME_PONG = 23,
};

enum management_dir_id {
    MGMT_DIR_ROOT = 1,
    MGMT_DIR_UPLOADED = 2,
    MGMT_DIR_TRASH = 3,
};

enum management_file_class {
    MGMT_FILE_CONFIG = 1,
    MGMT_FILE_ROOT_SST = 2,
    MGMT_FILE_UPLOADED_SST = 3,
    MGMT_FILE_TRASH_SST = 4,
};

enum management_result_code {
    MGMT_RESULT_OK = 0,
    MGMT_RESULT_INVALID_REQUEST = -1,
    MGMT_RESULT_NOT_FOUND = -2,
    MGMT_RESULT_BUSY = -3,
    MGMT_RESULT_IO_ERROR = -4,
    MGMT_RESULT_VALIDATION_ERROR = -5,
    MGMT_RESULT_UNSUPPORTED_TARGET = -6,
    MGMT_RESULT_INTERNAL_ERROR = -7,
};

enum management_session_state {
    MGMT_SESSION_IDLE = 0,
    MGMT_SESSION_LIST_DIR = 1,
    MGMT_SESSION_SEND_FILE = 2,
    MGMT_SESSION_BEGIN_UPLOAD = 3,
    MGMT_SESSION_COMMIT_UPLOAD = 4,
    MGMT_SESSION_TRASH_FILE = 5,
    MGMT_SESSION_SEND_ACTION_RESULT = 6,
    MGMT_SESSION_MARK_SST_UPLOADED = 7,
};

struct management_frame_header {
    uint32_t magic;
    uint16_t version;
    uint16_t frame_type;
    uint32_t request_id;
    uint32_t payload_length;
} __attribute__((packed));

_Static_assert(sizeof(struct management_frame_header) == MANAGEMENT_PROTOCOL_FRAME_HEADER_SIZE,
               "Unexpected management frame header size");

struct management_list_dir_req {
    uint16_t dir_id;
    uint16_t reserved;
} __attribute__((packed));

struct management_get_file_req {
    uint16_t file_class;
    uint16_t reserved;
    int32_t record_id;
} __attribute__((packed));

struct management_trash_file_req {
    int32_t record_id;
} __attribute__((packed));

struct management_mark_sst_uploaded_req {
    int32_t record_id;
} __attribute__((packed));

struct management_put_file_begin_req {
    uint16_t file_class;
    uint16_t reserved;
    uint64_t file_size;
} __attribute__((packed));

struct management_set_time_req {
    uint32_t utc_seconds;
    uint32_t micros;
} __attribute__((packed));

struct management_list_dir_entry_frame {
    uint16_t dir_id;
    uint16_t file_class;
    int32_t record_id;
    uint64_t file_size;
    int64_t timestamp_utc;
    uint32_t duration_ms;
    uint8_t sst_version;
    char name[MANAGEMENT_PROTOCOL_NAME_LENGTH];
} __attribute__((packed));

struct management_list_dir_done_frame {
    uint32_t entry_count;
} __attribute__((packed));

struct management_file_begin_frame {
    uint16_t file_class;
    uint16_t reserved;
    int32_t record_id;
    uint64_t file_size;
    uint32_t max_chunk_payload;
    char name[MANAGEMENT_PROTOCOL_NAME_LENGTH];
} __attribute__((packed));

struct management_action_result_frame {
    int32_t result_code;
} __attribute__((packed));

struct management_error_frame {
    int32_t error_code;
} __attribute__((packed));

struct management_session {
    enum management_session_state state;
    uint32_t active_request_id;
    uint32_t response_request_id;
    int32_t action_result_code;

    uint16_t list_dir_id;
    uint32_t list_entry_count;
    bool list_include_config_pending;
    bool list_dir_open;
    bool list_entry_pending;
    DIR list_dir;
    FILINFO list_fno;
    struct management_list_dir_entry_frame pending_list_entry;

    bool diag_list_active;
    uint32_t diag_list_request_id;
    uint64_t diag_list_start_us;
    uint64_t diag_list_scan_us;
    uint32_t diag_list_files_visited;
    uint32_t diag_list_config_entries;
    uint32_t diag_list_sst_scans;

    bool file_open;
    FIL file;
    uint16_t file_class;
    int32_t file_record_id;
    uint64_t file_size;
    uint64_t file_offset;
    bool file_begin_sent;
    char file_name[MANAGEMENT_PROTOCOL_NAME_LENGTH];

    bool diag_file_active;
    uint32_t diag_file_request_id;
    uint64_t diag_file_start_us;
    uint64_t diag_file_last_tx_success_us;
    uint64_t diag_file_max_tx_gap_us;
    uint64_t diag_file_max_read_us;
    uint64_t diag_file_bytes_read;
    uint64_t diag_file_tx_bytes;
    uint32_t diag_file_chunks_sent;
    uint32_t diag_file_read_calls;
    uint32_t diag_file_min_sndbuf;
    uint32_t diag_file_sndbuf_blocked;
    uint32_t diag_file_write_errors;

    volatile bool tx_ack_wakeup_pending;
    uint32_t tx_ack_events;
    uint64_t tx_acked_bytes;
    uint16_t tx_last_ack_len;
    uint64_t tx_last_ack_us;

    bool upload_open;
    FIL upload_file;
    uint32_t upload_request_id;
    uint16_t upload_file_class;
    uint64_t upload_expected_size;
    uint64_t upload_received_size;
    struct config upload_snapshot;

    bool pending_chunk_ready;
    uint16_t pending_chunk_length;
    uint8_t pending_chunk_data[MANAGEMENT_PROTOCOL_IO_BUFFER_SIZE];

    int32_t pending_trash_record_id;
    int32_t pending_mark_uploaded_record_id;
};

void management_protocol_reset_session(struct tcpserver *server);
bool management_protocol_process_rx(struct tcpserver *server);
bool management_protocol_service(struct tcpserver *server);
bool management_protocol_needs_service(const struct tcpserver *server);

extern const struct tcpserver_protocol_ops management_protocol_ops;

#endif // MANAGEMENT_PROTOCOL_H