#include "management_protocol.h"

#include "tcpserver.h"

#include "../fw/sst.h"
#include "../ntp/ntp.h"
#include "../util/log.h"

#include "ff_stdio.h"
#include "lwip/tcp.h"
#include "pico/cyw43_arch.h"
#include "pico/time.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MANAGEMENT_TX_MAX_FRAMES_PER_TURN 16u
#define MANAGEMENT_TX_MAX_BYTES_PER_TURN  TCP_SND_BUF

static uint8_t management_tx_frame_buffer[MGMT_V2_TX_FRAME_BUFFER_SIZE];

static bool management_protocol_can_accept(const struct tcpserver *server);
static void management_protocol_on_accept(struct tcpserver *server);
static void management_protocol_on_disconnect(struct tcpserver *server);
static bool management_protocol_detect(const struct tcpserver *server);

static void management_diag_log_file_summary(struct management_session *session, const char *reason);
static void management_diag_log_list_summary(struct management_session *session, const char *reason);

static void management_diag_begin_file(struct management_session *session, uint32_t request_id) {
    session->diag_file_active = true;
    session->diag_file_request_id = request_id;
    session->diag_file_start_us = time_us_64();
    session->diag_file_last_tx_success_us = 0;
    session->diag_file_max_tx_gap_us = 0;
    session->diag_file_max_read_us = 0;
    session->diag_file_bytes_read = 0;
    session->diag_file_tx_bytes = 0;
    session->diag_file_chunks_sent = 0;
    session->diag_file_read_calls = 0;
    session->diag_file_min_sndbuf = UINT32_MAX;
    session->diag_file_sndbuf_blocked = 0;
    session->diag_file_write_errors = 0;
}

static void management_diag_begin_list(struct management_session *session, uint32_t request_id) {
    session->diag_list_active = true;
    session->diag_list_request_id = request_id;
    session->diag_list_start_us = time_us_64();
    session->diag_list_scan_us = 0;
    session->diag_list_files_visited = 0;
    session->diag_list_config_entries = 0;
    session->diag_list_sst_scans = 0;
}

static void management_diag_note_tx_success(struct management_session *session, uint16_t frame_type,
                                            uint32_t total_bytes, uint32_t sndbuf_before) {
    uint64_t now_us;
    uint64_t gap_us;

    if (!session->diag_file_active) {
        return;
    }

    if (!(frame_type == MGMT_FRAME_FILE_BEGIN || frame_type == MGMT_FRAME_FILE_CHUNK ||
          frame_type == MGMT_FRAME_FILE_END)) {
        return;
    }

    if (sndbuf_before < session->diag_file_min_sndbuf) {
        session->diag_file_min_sndbuf = sndbuf_before;
    }

    now_us = time_us_64();
    if (session->diag_file_last_tx_success_us != 0u) {
        gap_us = now_us - session->diag_file_last_tx_success_us;
        if (gap_us > session->diag_file_max_tx_gap_us) {
            session->diag_file_max_tx_gap_us = gap_us;
        }
    }
    session->diag_file_last_tx_success_us = now_us;
    session->diag_file_tx_bytes += total_bytes;
    if (frame_type == MGMT_FRAME_FILE_CHUNK) {
        session->diag_file_chunks_sent++;
    }
}

static void management_diag_note_tx_blocked(struct management_session *session, uint16_t frame_type,
                                            uint32_t sndbuf_before) {
    if (!session->diag_file_active) {
        return;
    }

    if (!(frame_type == MGMT_FRAME_FILE_BEGIN || frame_type == MGMT_FRAME_FILE_CHUNK ||
          frame_type == MGMT_FRAME_FILE_END)) {
        return;
    }

    if (sndbuf_before < session->diag_file_min_sndbuf) {
        session->diag_file_min_sndbuf = sndbuf_before;
    }
    session->diag_file_sndbuf_blocked++;
}

static void management_diag_note_tx_error(struct management_session *session, uint16_t frame_type) {
    if (!session->diag_file_active) {
        return;
    }

    if (frame_type == MGMT_FRAME_FILE_BEGIN || frame_type == MGMT_FRAME_FILE_CHUNK ||
        frame_type == MGMT_FRAME_FILE_END) {
        session->diag_file_write_errors++;
    }
}

const struct tcpserver_protocol_ops management_protocol_ops = {
    .can_accept = management_protocol_can_accept,
    .on_accept = management_protocol_on_accept,
    .on_disconnect = management_protocol_on_disconnect,
    .detect = management_protocol_detect,
    .process_rx = management_protocol_process_rx,
    .service = management_protocol_service,
    .needs_service = management_protocol_needs_service,
};

static bool management_send_frame_with_flush(struct tcpserver *server, uint16_t frame_type, uint32_t request_id,
                                             const void *payload, uint32_t payload_length, bool flush_now) {
    struct management_frame_header header = {
        .magic = MANAGEMENT_PROTOCOL_MAGIC,
        .version = MANAGEMENT_PROTOCOL_VERSION,
        .frame_type = frame_type,
        .request_id = request_id,
        .payload_length = payload_length,
    };
    uint32_t total_bytes = sizeof(header) + payload_length;
    err_t err;
    uint32_t sndbuf;

    if (server->client_pcb == NULL || total_bytes > sizeof(management_tx_frame_buffer)) {
        return false;
    }

    memcpy(management_tx_frame_buffer, &header, sizeof(header));
    if (payload_length > 0u) {
        memcpy(management_tx_frame_buffer + sizeof(header), payload, payload_length);
    }

    cyw43_arch_lwip_begin();
    sndbuf = tcp_sndbuf(server->client_pcb);
    if (sndbuf < total_bytes) {
        management_diag_note_tx_blocked(&server->management, frame_type, sndbuf);
        cyw43_arch_lwip_end();
        return false;
    }

    err = tcp_write(server->client_pcb, management_tx_frame_buffer, total_bytes, TCP_WRITE_FLAG_COPY);
    if (err == ERR_OK) {
        if (flush_now) {
            tcp_output(server->client_pcb);
        }
        management_diag_note_tx_success(&server->management, frame_type, total_bytes, sndbuf);
    } else {
        management_diag_note_tx_error(&server->management, frame_type);
    }
    cyw43_arch_lwip_end();

    return err == ERR_OK;
}

static bool management_send_frame(struct tcpserver *server, uint16_t frame_type, uint32_t request_id,
                                  const void *payload, uint32_t payload_length) {
    return management_send_frame_with_flush(server, frame_type, request_id, payload, payload_length, true);
}

static FSIZE_t management_get_file_size(const char *path) {
    FILINFO finfo;

    if (f_stat(path, &finfo) != FR_OK) {
        return 0;
    }

    return finfo.fsize;
}

static void management_flush_output(struct tcpserver *server) {
    if (server->client_pcb == NULL) {
        return;
    }

    cyw43_arch_lwip_begin();
    tcp_output(server->client_pcb);
    cyw43_arch_lwip_end();
}

static void management_diag_log_file_summary(struct management_session *session, const char *reason) {
    uint64_t elapsed_us;
    uint32_t min_sndbuf;

    if (!session->diag_file_active) {
        return;
    }

    elapsed_us = time_us_64() - session->diag_file_start_us;
    min_sndbuf = session->diag_file_min_sndbuf == UINT32_MAX ? 0u : session->diag_file_min_sndbuf;
    LOG("MGMT",
        "GET_FILE %s req=%u class=%u rec=%ld size=%llu off=%llu elapsed_us=%llu tx_bytes=%llu rd_bytes=%llu chunks=%u reads=%u max_read_us=%llu min_sndbuf=%u sndblk=%u wrerr=%u max_tx_gap_us=%llu\n",
        reason != NULL ? reason : "summary", (unsigned)session->diag_file_request_id,
        (unsigned)session->file_class, (long)session->file_record_id, (unsigned long long)session->file_size,
        (unsigned long long)session->file_offset, (unsigned long long)elapsed_us,
        (unsigned long long)session->diag_file_tx_bytes, (unsigned long long)session->diag_file_bytes_read,
        (unsigned)session->diag_file_chunks_sent, (unsigned)session->diag_file_read_calls,
        (unsigned long long)session->diag_file_max_read_us, (unsigned)min_sndbuf,
        (unsigned)session->diag_file_sndbuf_blocked, (unsigned)session->diag_file_write_errors,
        (unsigned long long)session->diag_file_max_tx_gap_us);
    session->diag_file_active = false;
}

static void management_diag_log_list_summary(struct management_session *session, const char *reason) {
    uint64_t elapsed_us;

    if (!session->diag_list_active) {
        return;
    }

    elapsed_us = time_us_64() - session->diag_list_start_us;
    LOG("MGMT",
        "LIST_DIR %s req=%u dir=%u entries=%u visited=%u config=%u scans=%u scan_us=%llu elapsed_us=%llu\n",
        reason != NULL ? reason : "summary", (unsigned)session->diag_list_request_id,
        (unsigned)session->list_dir_id, (unsigned)session->list_entry_count,
        (unsigned)session->diag_list_files_visited, (unsigned)session->diag_list_config_entries,
        (unsigned)session->diag_list_sst_scans, (unsigned long long)session->diag_list_scan_us,
        (unsigned long long)elapsed_us);
    session->diag_list_active = false;
}

static void management_request_close(struct tcpserver *server, int32_t error_code) {
    server->last_error = error_code;
    server->close_client_requested = true;
}

static bool management_send_error(struct tcpserver *server, uint32_t request_id, int32_t error_code) {
    LOG("MGMT", "Sending error %d for request %u\n", (int)error_code, (unsigned)request_id);
    struct management_error_frame frame = {.error_code = error_code};
    return management_send_frame(server, MGMT_FRAME_ERROR, request_id, &frame, sizeof(frame));
}

static bool management_send_request_error(struct tcpserver *server, uint32_t request_id, int32_t error_code) {
    if (management_send_error(server, request_id, error_code)) {
        return true;
    }

    management_request_close(server, error_code);
    return false;
}

static void management_protocol_violation(struct tcpserver *server, uint32_t request_id, int32_t error_code) {
    (void)management_send_error(server, request_id, error_code);
    management_request_close(server, error_code);
}

static void management_close_dir(struct management_session *session) {
    if (session->list_dir_open) {
        f_closedir(&session->list_dir);
        session->list_dir_open = false;
    }
    session->list_entry_pending = false;
}

static void management_close_file(struct management_session *session) {
    if (session->file_open) {
        f_close(&session->file);
        session->file_open = false;
    }
    session->file_size = 0;
    session->file_offset = 0;
    session->file_begin_sent = false;
    session->pending_chunk_ready = false;
    session->pending_chunk_length = 0;
    memset(session->file_name, 0, sizeof(session->file_name));
}

static void management_abort_upload(struct management_session *session, bool remove_staging) {
    if (session->upload_open) {
        f_close(&session->upload_file);
        session->upload_open = false;
    }
    if (remove_staging) {
        f_unlink(MANAGEMENT_PROTOCOL_CONFIG_STAGING_PATH);
    }

    session->upload_request_id = 0;
    session->upload_file_class = 0;
    session->upload_expected_size = 0;
    session->upload_received_size = 0;
    session->pending_chunk_ready = false;
    session->pending_chunk_length = 0;
}

static bool management_write_upload_chunk(struct management_session *session, const void *payload,
                                          uint32_t payload_length, int32_t *error_code) {
    UINT bytes_written = 0;

    if (!session->upload_open || payload == NULL) {
        if (error_code != NULL) {
            *error_code = MGMT_RESULT_INVALID_REQUEST;
        }
        return false;
    }

    if (payload_length == 0u) {
        return true;
    }

    if (f_write(&session->upload_file, payload, payload_length, &bytes_written) != FR_OK ||
        bytes_written != payload_length) {
        management_abort_upload(session, true);
        if (error_code != NULL) {
            *error_code = MGMT_RESULT_IO_ERROR;
        }
        return false;
    }

    session->upload_received_size += payload_length;
    return true;
}

static const char *management_dir_path(uint16_t dir_id) {
    switch (dir_id) {
        case MGMT_DIR_ROOT:
            return "";
        case MGMT_DIR_UPLOADED:
            return "uploaded";
        case MGMT_DIR_TRASH:
            return "trash";
        default:
            return NULL;
    }
}

static uint16_t management_file_class_for_dir(uint16_t dir_id) {
    switch (dir_id) {
        case MGMT_DIR_ROOT:
            return MGMT_FILE_ROOT_SST;
        case MGMT_DIR_UPLOADED:
            return MGMT_FILE_UPLOADED_SST;
        case MGMT_DIR_TRASH:
            return MGMT_FILE_TRASH_SST;
        default:
            return 0;
    }
}

static bool management_make_record_name(int32_t record_id, char name[MANAGEMENT_PROTOCOL_NAME_LENGTH]) {
    int written;

    if (record_id < 0 || record_id > 99999) {
        return false;
    }

    memset(name, 0, MANAGEMENT_PROTOCOL_NAME_LENGTH);
    written = snprintf(name, MANAGEMENT_PROTOCOL_NAME_LENGTH, "%05ld.SST", (long)record_id);
    return written > 0 && written < (int)MANAGEMENT_PROTOCOL_NAME_LENGTH;
}

static bool management_build_sst_path(uint16_t file_class, int32_t record_id, char *path, size_t path_size,
                                      char name[MANAGEMENT_PROTOCOL_NAME_LENGTH]) {
    if (!management_make_record_name(record_id, name)) {
        return false;
    }

    switch (file_class) {
        case MGMT_FILE_ROOT_SST:
            return snprintf(path, path_size, "%s", name) > 0;
        case MGMT_FILE_UPLOADED_SST:
            return snprintf(path, path_size, "uploaded/%s", name) > 0;
        case MGMT_FILE_TRASH_SST:
            return snprintf(path, path_size, "trash/%s", name) > 0;
        default:
            return false;
    }
}

static bool management_build_config_entry(struct management_list_dir_entry_frame *entry) {
    FILINFO finfo;

    if (f_stat(MANAGEMENT_PROTOCOL_CONFIG_NAME, &finfo) != FR_OK) {
        return false;
    }

    memset(entry, 0, sizeof(*entry));
    entry->dir_id = MGMT_DIR_ROOT;
    entry->file_class = MGMT_FILE_CONFIG;
    entry->record_id = 0;
    entry->file_size = finfo.fsize;
    memcpy(entry->name, MANAGEMENT_PROTOCOL_CONFIG_NAME, strlen(MANAGEMENT_PROTOCOL_CONFIG_NAME));
    return true;
}

static bool management_parse_record_id(const char *filename, int32_t *record_id) {
    char digits[6];

    if (filename == NULL || record_id == NULL || strlen(filename) != 9u || strcmp(filename + 5, ".SST") != 0) {
        return false;
    }

    for (uint32_t index = 0; index < 5u; ++index) {
        if (filename[index] < '0' || filename[index] > '9') {
            return false;
        }
        digits[index] = filename[index];
    }

    digits[5] = 0;
    *record_id = (int32_t)strtol(digits, NULL, 10);
    return true;
}

static bool management_build_sst_entry(uint16_t dir_id, const char *filename,
                                       struct management_list_dir_entry_frame *entry) {
    char path[24];
    const char *dir_path = management_dir_path(dir_id);
    struct sst_file_info info;
    int32_t record_id;
    int written;

    if (dir_path == NULL || !management_parse_record_id(filename, &record_id)) {
        return false;
    }

    if (dir_path[0] == 0) {
        written = snprintf(path, sizeof(path), "%s", filename);
    } else {
        written = snprintf(path, sizeof(path), "%s/%s", dir_path, filename);
    }
    if (written <= 0 || written >= (int)sizeof(path)) {
        return false;
    }

    info = sst_get_file_info(path);
    memset(entry, 0, sizeof(*entry));
    entry->dir_id = dir_id;
    entry->file_class = management_file_class_for_dir(dir_id);
    entry->record_id = record_id;
    entry->file_size = management_get_file_size(path);
    entry->timestamp_utc = (int64_t)info.timestamp;
    entry->duration_ms = info.duration_ms;
    entry->sst_version = info.version;
    memcpy(entry->name, filename, strlen(filename));
    return true;
}

static bool management_open_requested_file(struct management_session *session, uint16_t file_class, int32_t record_id,
                                           int32_t *error_code) {
    char path[24];
    FILINFO finfo;
    FRESULT fr;

    management_close_file(session);

    memset(path, 0, sizeof(path));
    memset(session->file_name, 0, sizeof(session->file_name));
    session->file_record_id = record_id;
    session->file_class = file_class;

    switch (file_class) {
        case MGMT_FILE_CONFIG:
            session->file_record_id = 0;
            memcpy(session->file_name, MANAGEMENT_PROTOCOL_CONFIG_NAME, strlen(MANAGEMENT_PROTOCOL_CONFIG_NAME));
            snprintf(path, sizeof(path), "%s", MANAGEMENT_PROTOCOL_CONFIG_NAME);
            break;
        case MGMT_FILE_ROOT_SST:
        case MGMT_FILE_UPLOADED_SST:
        case MGMT_FILE_TRASH_SST:
            if (!management_build_sst_path(file_class, record_id, path, sizeof(path), session->file_name)) {
                *error_code = MGMT_RESULT_INVALID_REQUEST;
                return false;
            }
            break;
        default:
            *error_code = MGMT_RESULT_UNSUPPORTED_TARGET;
            return false;
    }

    fr = f_stat(path, &finfo);
    if (fr != FR_OK) {
        *error_code = MGMT_RESULT_NOT_FOUND;
        return false;
    }

    fr = f_open(&session->file, path, FA_OPEN_EXISTING | FA_READ);
    if (!(fr == FR_OK || fr == FR_EXIST)) {
        *error_code = MGMT_RESULT_IO_ERROR;
        return false;
    }

    session->file_open = true;
    session->file_size = finfo.fsize;
    session->file_offset = 0;
    session->file_begin_sent = false;
    session->pending_chunk_ready = false;
    session->pending_chunk_length = 0;
    return true;
}

static void management_begin_action_result(struct management_session *session, uint32_t request_id,
                                           int32_t result_code) {
    session->response_request_id = request_id;
    session->action_result_code = result_code;
    session->state = MGMT_SESSION_SEND_ACTION_RESULT;
}

static bool management_queue_next_list_entry(struct management_session *session) {
    while (session->list_fno.fname[0] != 0) {
        struct management_list_dir_entry_frame entry;
        FRESULT fr;
        uint64_t scan_start_us;
        bool entry_built;

        session->diag_list_files_visited++;
        scan_start_us = time_us_64();
        entry_built = management_build_sst_entry(session->list_dir_id, session->list_fno.fname, &entry);
        session->diag_list_sst_scans++;
        session->diag_list_scan_us += time_us_64() - scan_start_us;

        if (entry_built) {
            session->pending_list_entry = entry;
            session->list_entry_pending = true;
            fr = f_findnext(&session->list_dir, &session->list_fno);
            if (fr != FR_OK) {
                session->list_fno.fname[0] = 0;
            }
            return true;
        }

        if (f_findnext(&session->list_dir, &session->list_fno) != FR_OK) {
            session->list_fno.fname[0] = 0;
        }
    }

    return false;
}

static bool management_service_list_dir(struct tcpserver *server) {
    struct management_session *session = &server->management;

    if (session->list_include_config_pending) {
        struct management_list_dir_entry_frame entry;

        session->list_include_config_pending = false;
        if (management_build_config_entry(&entry)) {
            if (!management_send_frame(server, MGMT_FRAME_LIST_DIR_ENTRY, session->active_request_id, &entry,
                                       sizeof(entry))) {
                session->list_include_config_pending = true;
                return false;
            }
            session->diag_list_config_entries++;
            session->list_entry_count++;
            return true;
        }
    }

    if (session->list_entry_pending) {
        if (!management_send_frame(server, MGMT_FRAME_LIST_DIR_ENTRY, session->active_request_id,
                                   &session->pending_list_entry, sizeof(session->pending_list_entry))) {
            return false;
        }
        session->list_entry_pending = false;
        session->list_entry_count++;
        return true;
    }

    if (!session->list_dir_open) {
        const char *dir_path = management_dir_path(session->list_dir_id);
        FRESULT fr;

        if (dir_path == NULL) {
            management_protocol_violation(server, session->active_request_id, MGMT_RESULT_INVALID_REQUEST);
            return false;
        }

        fr = f_findfirst(&session->list_dir, &session->list_fno, dir_path, "?????.SST");
        if (fr == FR_OK) {
            session->list_dir_open = true;
        } else if (fr == FR_NO_PATH || fr == FR_NO_FILE) {
            session->list_fno.fname[0] = 0;
        } else {
            management_protocol_violation(server, session->active_request_id, MGMT_RESULT_IO_ERROR);
            return false;
        }
    }

    if (session->list_dir_open && management_queue_next_list_entry(session)) {
        return true;
    }

    if (!management_send_frame(server, MGMT_FRAME_LIST_DIR_DONE, session->active_request_id,
                               &(struct management_list_dir_done_frame){.entry_count = session->list_entry_count},
                               sizeof(struct management_list_dir_done_frame))) {
        return false;
    }

    management_close_dir(session);
    management_diag_log_list_summary(session, "done");
    session->active_request_id = 0;
    session->list_entry_count = 0;
    session->state = MGMT_SESSION_IDLE;
    return true;
}

static bool management_service_send_file(struct tcpserver *server) {
    struct management_session *session = &server->management;
    uint32_t frames_sent = 0;
    uint32_t bytes_queued = 0;
    bool made_progress = false;
    bool output_needed = false;

    if (!session->file_open) {
        management_protocol_violation(server, session->active_request_id, MGMT_RESULT_INTERNAL_ERROR);
        return false;
    }

    while (frames_sent < MANAGEMENT_TX_MAX_FRAMES_PER_TURN && bytes_queued < MANAGEMENT_TX_MAX_BYTES_PER_TURN) {
        uint64_t remaining_bytes;

        if (!session->file_begin_sent) {
            struct management_file_begin_frame begin = {
                .file_class = session->file_class,
                .record_id = session->file_record_id,
                .file_size = session->file_size,
                .max_chunk_payload = MGMT_V2_MAX_FILE_CHUNK_PAYLOAD,
            };

            memcpy(begin.name, session->file_name, sizeof(begin.name));
            if (!management_send_frame_with_flush(server, MGMT_FRAME_FILE_BEGIN, session->active_request_id, &begin,
                                                  sizeof(begin), false)) {
                break;
            }
            session->file_begin_sent = true;
            frames_sent++;
            bytes_queued += sizeof(struct management_frame_header) + sizeof(begin);
            made_progress = true;
            output_needed = true;
            continue;
        }

        remaining_bytes = session->file_size - session->file_offset;
        if (!session->pending_chunk_ready && remaining_bytes > 0u) {
            UINT bytes_read = 0;
            uint32_t to_read =
                remaining_bytes > MANAGEMENT_PROTOCOL_IO_BUFFER_SIZE ? MANAGEMENT_PROTOCOL_IO_BUFFER_SIZE
                                                                     : (uint32_t)remaining_bytes;
            uint64_t read_start_us = time_us_64();
            FRESULT read_result;
            uint64_t read_elapsed_us;

            read_result = f_read(&session->file, session->pending_chunk_data, to_read, &bytes_read);
            read_elapsed_us = time_us_64() - read_start_us;
            session->diag_file_read_calls++;
            if (read_elapsed_us > session->diag_file_max_read_us) {
                session->diag_file_max_read_us = read_elapsed_us;
            }

            if (read_result != FR_OK || bytes_read == 0u) {
                if (output_needed) {
                    management_flush_output(server);
                }
                management_protocol_violation(server, session->active_request_id, MGMT_RESULT_IO_ERROR);
                return made_progress;
            }

            session->diag_file_bytes_read += bytes_read;
            session->pending_chunk_ready = true;
            session->pending_chunk_length = (uint16_t)bytes_read;
        }

        if (session->pending_chunk_ready) {
            uint16_t pending_length = session->pending_chunk_length;

            if (!management_send_frame_with_flush(server, MGMT_FRAME_FILE_CHUNK, session->active_request_id,
                                                  session->pending_chunk_data, pending_length, false)) {
                break;
            }
            session->file_offset += pending_length;
            session->pending_chunk_ready = false;
            session->pending_chunk_length = 0;
            frames_sent++;
            bytes_queued += sizeof(struct management_frame_header) + pending_length;
            made_progress = true;
            output_needed = true;
            continue;
        }

        if (!management_send_frame_with_flush(server, MGMT_FRAME_FILE_END, session->active_request_id, NULL, 0,
                                              false)) {
            break;
        }

        frames_sent++;
        bytes_queued += sizeof(struct management_frame_header);
        made_progress = true;
        output_needed = true;
        management_flush_output(server);
        management_diag_log_file_summary(session, "done");
        management_close_file(session);
        session->active_request_id = 0;
        session->state = MGMT_SESSION_IDLE;
        return true;
    }

    if (output_needed) {
        management_flush_output(server);
    }

    return made_progress;
}

static bool management_service_begin_upload(struct tcpserver *server) {
    struct management_session *session = &server->management;
    FRESULT fr;

    f_unlink(MANAGEMENT_PROTOCOL_CONFIG_STAGING_PATH);

    if (session->upload_file_class != MGMT_FILE_CONFIG) {
        management_begin_action_result(session, session->active_request_id, MGMT_RESULT_UNSUPPORTED_TARGET);
        return true;
    }

    fr = f_open(&session->upload_file, MANAGEMENT_PROTOCOL_CONFIG_STAGING_PATH, FA_CREATE_ALWAYS | FA_WRITE);
    if (fr != FR_OK) {
        management_begin_action_result(session, session->active_request_id, MGMT_RESULT_IO_ERROR);
        return true;
    }

    session->upload_open = true;
    session->upload_request_id = session->active_request_id;
    management_begin_action_result(session, session->active_request_id, MGMT_RESULT_OK);
    return true;
}

static bool management_service_trash_file(struct tcpserver *server) {
    struct management_session *session = &server->management;
    char old_path[16];
    char new_path[24];

    if (!management_make_record_name(session->pending_trash_record_id, session->file_name)) {
        management_begin_action_result(session, session->active_request_id, MGMT_RESULT_INVALID_REQUEST);
        return true;
    }

    snprintf(old_path, sizeof(old_path), "%s", session->file_name);
    snprintf(new_path, sizeof(new_path), "trash/%s", session->file_name);

    if (f_stat(old_path, NULL) != FR_OK) {
        management_begin_action_result(session, session->active_request_id, MGMT_RESULT_NOT_FOUND);
        return true;
    }

    if (ff_rename(old_path, new_path, 1) != 0) {
        management_begin_action_result(session, session->active_request_id, MGMT_RESULT_IO_ERROR);
        return true;
    }

    management_begin_action_result(session, session->active_request_id, MGMT_RESULT_OK);
    return true;
}

static bool management_service_mark_sst_uploaded(struct tcpserver *server) {
    struct management_session *session = &server->management;
    char old_path[16];
    char new_path[24];

    if (!management_make_record_name(session->pending_mark_uploaded_record_id, session->file_name)) {
        management_begin_action_result(session, session->active_request_id, MGMT_RESULT_INVALID_REQUEST);
        return true;
    }

    snprintf(old_path, sizeof(old_path), "%s", session->file_name);
    snprintf(new_path, sizeof(new_path), "uploaded/%s", session->file_name);

    if (f_stat(old_path, NULL) != FR_OK) {
        management_begin_action_result(session, session->active_request_id, MGMT_RESULT_NOT_FOUND);
        return true;
    }

    if (ff_rename(old_path, new_path, 1) != 0) {
        management_begin_action_result(session, session->active_request_id, MGMT_RESULT_IO_ERROR);
        return true;
    }

    management_begin_action_result(session, session->active_request_id, MGMT_RESULT_OK);
    return true;
}

static bool management_service_commit_upload(struct tcpserver *server) {
    struct management_session *session = &server->management;

    if (!session->upload_open || session->upload_request_id != session->active_request_id) {
        management_begin_action_result(session, session->active_request_id, MGMT_RESULT_INVALID_REQUEST);
        return true;
    }

    if (session->upload_received_size != session->upload_expected_size) {
        management_abort_upload(session, true);
        management_begin_action_result(session, session->active_request_id, MGMT_RESULT_INVALID_REQUEST);
        return true;
    }

    f_close(&session->upload_file);
    session->upload_open = false;

    if (!config_load_file(MANAGEMENT_PROTOCOL_CONFIG_STAGING_PATH, &session->upload_snapshot)) {
        management_abort_upload(session, true);
        management_begin_action_result(session, session->active_request_id, MGMT_RESULT_VALIDATION_ERROR);
        return true;
    }

    if (!config_commit_staged_file(MANAGEMENT_PROTOCOL_CONFIG_STAGING_PATH)) {
        management_abort_upload(session, true);
        management_begin_action_result(session, session->active_request_id, MGMT_RESULT_IO_ERROR);
        return true;
    }

    session->upload_request_id = 0;
    config_apply_snapshot(&session->upload_snapshot);
    management_begin_action_result(session, session->active_request_id, MGMT_RESULT_OK);
    return true;
}

static bool management_service_send_action_result(struct tcpserver *server) {
    struct management_session *session = &server->management;
    struct management_action_result_frame frame = {.result_code = session->action_result_code};

    if (!management_send_frame(server, MGMT_FRAME_ACTION_RESULT, session->response_request_id, &frame, sizeof(frame))) {
        return false;
    }

    session->active_request_id = 0;
    session->response_request_id = 0;
    session->action_result_code = MGMT_RESULT_OK;
    session->state = MGMT_SESSION_IDLE;
    return true;
}

void management_protocol_reset_session(struct tcpserver *server) {
    memset(&server->management, 0, sizeof(server->management));
}

static bool management_protocol_can_accept(const struct tcpserver *server) {
    (void)server;
    return true;
}

static void management_protocol_on_accept(struct tcpserver *server) { management_protocol_reset_session(server); }

static void management_protocol_on_disconnect(struct tcpserver *server) {
    management_diag_log_list_summary(&server->management, "disconnect");
    management_diag_log_file_summary(&server->management, "disconnect");
    management_close_dir(&server->management);
    management_close_file(&server->management);
    management_abort_upload(&server->management, true);
    management_protocol_reset_session(server);
}

static bool management_protocol_detect(const struct tcpserver *server) {
    uint32_t magic;

    if (server->rx_len < sizeof(magic)) {
        return false;
    }

    memcpy(&magic, server->rx_buffer, sizeof(magic));
    return magic == MANAGEMENT_PROTOCOL_MAGIC;
}

bool management_protocol_process_rx(struct tcpserver *server) {
    struct management_session *session = &server->management;

    while (server->rx_len >= sizeof(struct management_frame_header)) {
        struct management_frame_header header;
        const uint8_t *payload;
        uint32_t frame_bytes;

        if (session->state != MGMT_SESSION_IDLE) {
            return true;
        }

        memcpy(&header, server->rx_buffer, sizeof(header));
        if (header.magic != MANAGEMENT_PROTOCOL_MAGIC || header.version != MANAGEMENT_PROTOCOL_VERSION) {
            management_protocol_violation(server, header.request_id, MGMT_RESULT_INVALID_REQUEST);
            return false;
        }

        frame_bytes = sizeof(struct management_frame_header) + header.payload_length;
        if (server->rx_len < frame_bytes) {
            return true;
        }

        payload = server->rx_buffer + sizeof(struct management_frame_header);

        switch (header.frame_type) {
            case MGMT_FRAME_LIST_DIR_REQ: {
                struct management_list_dir_req req;

                if (header.payload_length != sizeof(req)) {
                    management_protocol_violation(server, header.request_id, MGMT_RESULT_INVALID_REQUEST);
                    return false;
                }

                memcpy(&req, payload, sizeof(req));
                tcpserver_consume_rx(server, (uint16_t)frame_bytes);

                if (session->upload_open) {
                    return management_send_request_error(server, header.request_id, MGMT_RESULT_BUSY);
                }

                if (management_dir_path(req.dir_id) == NULL) {
                    return management_send_request_error(server, header.request_id, MGMT_RESULT_INVALID_REQUEST);
                }

                session->active_request_id = header.request_id;
                session->list_dir_id = req.dir_id;
                session->list_entry_count = 0;
                session->list_include_config_pending = req.dir_id == MGMT_DIR_ROOT;
                management_diag_begin_list(session, header.request_id);
                session->state = MGMT_SESSION_LIST_DIR;
                return true;
            }
            case MGMT_FRAME_GET_FILE_REQ: {
                struct management_get_file_req req;
                int32_t error_code = MGMT_RESULT_OK;

                if (header.payload_length != sizeof(req)) {
                    management_protocol_violation(server, header.request_id, MGMT_RESULT_INVALID_REQUEST);
                    return false;
                }

                memcpy(&req, payload, sizeof(req));
                tcpserver_consume_rx(server, (uint16_t)frame_bytes);

                if (session->upload_open) {
                    return management_send_request_error(server, header.request_id, MGMT_RESULT_BUSY);
                }

                if (!management_open_requested_file(session, req.file_class, req.record_id, &error_code)) {
                    return management_send_request_error(server, header.request_id, error_code);
                }

                session->active_request_id = header.request_id;
                management_diag_begin_file(session, header.request_id);
                session->state = MGMT_SESSION_SEND_FILE;
                return true;
            }
            case MGMT_FRAME_TRASH_FILE_REQ: {
                struct management_trash_file_req req;

                if (header.payload_length != sizeof(req)) {
                    management_protocol_violation(server, header.request_id, MGMT_RESULT_INVALID_REQUEST);
                    return false;
                }

                if (session->upload_open) {
                    management_begin_action_result(session, header.request_id, MGMT_RESULT_BUSY);
                    tcpserver_consume_rx(server, (uint16_t)frame_bytes);
                    return true;
                }

                memcpy(&req, payload, sizeof(req));
                session->active_request_id = header.request_id;
                session->pending_trash_record_id = req.record_id;
                session->state = MGMT_SESSION_TRASH_FILE;
                tcpserver_consume_rx(server, (uint16_t)frame_bytes);
                return true;
            }
            case MGMT_FRAME_MARK_SST_UPLOADED_REQ: {
                struct management_mark_sst_uploaded_req req;

                if (header.payload_length != sizeof(req)) {
                    management_protocol_violation(server, header.request_id, MGMT_RESULT_INVALID_REQUEST);
                    return false;
                }

                if (session->upload_open) {
                    management_begin_action_result(session, header.request_id, MGMT_RESULT_BUSY);
                    tcpserver_consume_rx(server, (uint16_t)frame_bytes);
                    return true;
                }

                memcpy(&req, payload, sizeof(req));
                session->active_request_id = header.request_id;
                session->pending_mark_uploaded_record_id = req.record_id;
                session->state = MGMT_SESSION_MARK_SST_UPLOADED;
                tcpserver_consume_rx(server, (uint16_t)frame_bytes);
                return true;
            }
            case MGMT_FRAME_PUT_FILE_BEGIN: {
                struct management_put_file_begin_req req;

                if (header.payload_length != sizeof(req)) {
                    management_protocol_violation(server, header.request_id, MGMT_RESULT_INVALID_REQUEST);
                    return false;
                }

                memcpy(&req, payload, sizeof(req));
                if (session->upload_open) {
                    management_begin_action_result(session, header.request_id, MGMT_RESULT_BUSY);
                } else if (req.file_class != MGMT_FILE_CONFIG) {
                    management_begin_action_result(session, header.request_id, MGMT_RESULT_UNSUPPORTED_TARGET);
                } else {
                    session->active_request_id = header.request_id;
                    session->upload_file_class = req.file_class;
                    session->upload_expected_size = req.file_size;
                    session->upload_received_size = 0;
                    session->state = MGMT_SESSION_BEGIN_UPLOAD;
                }

                tcpserver_consume_rx(server, (uint16_t)frame_bytes);
                return true;
            }
            case MGMT_FRAME_PUT_FILE_CHUNK: {
                int32_t error_code = MGMT_RESULT_OK;

                if (!session->upload_open || session->upload_request_id != header.request_id ||
                    header.payload_length > MANAGEMENT_PROTOCOL_MAX_UPLOAD_CHUNK_PAYLOAD_SIZE ||
                    (session->upload_received_size + header.payload_length) > session->upload_expected_size) {
                    management_protocol_violation(server, header.request_id, MGMT_RESULT_INVALID_REQUEST);
                    return false;
                }

                if (!management_write_upload_chunk(session, payload, header.payload_length, &error_code)) {
                    tcpserver_consume_rx(server, (uint16_t)frame_bytes);
                    return management_send_request_error(server, header.request_id, error_code);
                }

                tcpserver_consume_rx(server, (uint16_t)frame_bytes);
                return true;
            }
            case MGMT_FRAME_PUT_FILE_COMMIT:
                if (header.payload_length != 0u || !session->upload_open ||
                    session->upload_request_id != header.request_id) {
                    management_protocol_violation(server, header.request_id, MGMT_RESULT_INVALID_REQUEST);
                    return false;
                }

                session->active_request_id = header.request_id;
                session->state = MGMT_SESSION_COMMIT_UPLOAD;
                tcpserver_consume_rx(server, (uint16_t)frame_bytes);
                return true;
            case MGMT_FRAME_SET_TIME_REQ:
                if (header.payload_length != sizeof(struct management_set_time_req)) {
                    management_protocol_violation(server, header.request_id, MGMT_RESULT_INVALID_REQUEST);
                    return false;
                }

                if (session->upload_open) {
                    management_begin_action_result(session, header.request_id, MGMT_RESULT_BUSY);
                } else {
                    struct management_set_time_req req;
                    int32_t result_code;

                    memcpy(&req, payload, sizeof(req));
                    result_code = set_system_time_utc((time_t)req.utc_seconds, req.micros) ? MGMT_RESULT_OK
                                                                                           : MGMT_RESULT_INTERNAL_ERROR;
                    management_begin_action_result(session, header.request_id, result_code);
                }

                tcpserver_consume_rx(server, (uint16_t)frame_bytes);
                return true;
            case MGMT_FRAME_PING:
                if (header.payload_length != 0u) {
                    management_protocol_violation(server, header.request_id, MGMT_RESULT_INVALID_REQUEST);
                    return false;
                }

                if (!management_send_frame(server, MGMT_FRAME_PONG, header.request_id, NULL, 0)) {
                    return true;
                }

                tcpserver_consume_rx(server, (uint16_t)frame_bytes);
                return true;
            default:
                management_protocol_violation(server, header.request_id, MGMT_RESULT_INVALID_REQUEST);
                return false;
        }
    }

    return true;
}

bool management_protocol_service(struct tcpserver *server) {
    switch (server->management.state) {
        case MGMT_SESSION_LIST_DIR:
            return management_service_list_dir(server);
        case MGMT_SESSION_SEND_FILE:
            return management_service_send_file(server);
        case MGMT_SESSION_BEGIN_UPLOAD:
            return management_service_begin_upload(server);
        case MGMT_SESSION_COMMIT_UPLOAD:
            return management_service_commit_upload(server);
        case MGMT_SESSION_TRASH_FILE:
            return management_service_trash_file(server);
        case MGMT_SESSION_MARK_SST_UPLOADED:
            return management_service_mark_sst_uploaded(server);
        case MGMT_SESSION_SEND_ACTION_RESULT:
            return management_service_send_action_result(server);
        case MGMT_SESSION_IDLE:
        default:
            break;
    }

    return false;
}

bool management_protocol_needs_service(const struct tcpserver *server) {
    return server->management.state != MGMT_SESSION_IDLE;
}