#include "data_storage.h"
#include "data_acquisition.h"

#include "core1_ipc.h"

#include "sensor_setup.h"
#include "sst.h"

#include "../ntp/ntp.h"
#include "../util/config.h"
#include "../util/log.h"

#include "ff.h"
#include "pico/multicore.h"
#include "pico/platform.h"
#include "pico/unique_id.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

static FIL recording;
static bool recording_open = false;

static void storage_send_event(enum storage_session_event event_id) {
    multicore_fifo_push_blocking(CORE1_FIFO_WORD(CORE1_FIFO_FAMILY_STORAGE_EVENT, event_id));
}

static FRESULT write_exact(const void *buf, UINT n) {
    UINT bw;
    FRESULT fr = f_write(&recording, buf, n, &bw);
    return fr != FR_OK ? fr : (bw == n ? FR_OK : FR_DISK_ERR);
}

static int fail_session(const char *what, FRESULT fr) {
    if (recording_open) {
        f_close(&recording);
        recording_open = false;
    }
    LOG("STORAGE", "%s failed: %d\n", what, fr);
    return storage_backend_status_from_fresult(fr);
}

int setup_storage(void) {
    static FATFS fs;
    FRESULT fr = f_mount(&fs, "", 1);
    if (fr != FR_OK) {
        LOG("STORAGE", "Failed to mount filesystem: %d\n", fr);
        return PICO_ERROR_GENERIC;
    }
    LOG("STORAGE", "Filesystem mounted\n");

    char board_id_str[2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES + 1];
    pico_get_unique_board_id_string(board_id_str, 2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES + 1);
    FIL f;
    uint btw;
    fr = f_open(&f, "BOARDID", FA_OPEN_ALWAYS | FA_WRITE);
    if (fr == FR_OK || fr == FR_EXIST) {
        f_write(&f, board_id_str, 2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES, &btw);
    }
    f_close(&f);

    fr = f_mkdir("uploaded");
    if (!(fr == FR_OK || fr == FR_EXIST)) {
        return PICO_ERROR_GENERIC;
    }

    fr = f_mkdir("trash");
    if (!(fr == FR_OK || fr == FR_EXIST)) {
        return PICO_ERROR_GENERIC;
    }

    return 0;
}

static int open_datafile(void) {
    uint16_t index = 1;
    FIL index_fil;
    FRESULT fr = f_open(&index_fil, "INDEX", FA_OPEN_EXISTING | FA_READ);
    if (fr == FR_OK || fr == FR_EXIST) {
        uint br;
        f_read(&index_fil, &index, 2, &br);
        if (br == 2) {
            index = index + 1;
        }
    }
    f_close(&index_fil);

    fr = f_open(&index_fil, "INDEX", FA_OPEN_ALWAYS | FA_WRITE);
    if (fr == FR_OK) {
        f_lseek(&index_fil, 0);
        uint bw;
        f_write(&index_fil, &index, 2, &bw);
        f_close(&index_fil);
    } else {
        return PICO_ERROR_GENERIC;
    }

    char filename[10];
    sprintf(filename, "%05u.SST", index);
    LOG("STORAGE", "Creating file: %s\n", filename);
    fr = f_open(&recording, filename, FA_CREATE_NEW | FA_WRITE);
    if (fr != FR_OK) {
        LOG("STORAGE", "Failed to open %s: %d\n", filename, fr);
        return storage_backend_status_from_fresult(fr);
    }
    recording_open = true;

    struct sst_header h = {"SST", 4, 0, rtc_timestamp()};
    fr = write_exact(&h, sizeof(h));

#if HAS_IMU
    struct chunk_header ch = {CHUNK_TYPE_RATES, 2 * sizeof(struct samplerate_record)};
#else
    struct chunk_header ch = {CHUNK_TYPE_RATES, 1 * sizeof(struct samplerate_record)};
#endif
    if (fr == FR_OK)
        fr = write_exact(&ch, sizeof(ch));

    struct samplerate_record re = {CHUNK_TYPE_TRAVEL, config.travel_sample_rate};
    if (fr == FR_OK)
        fr = write_exact(&re, sizeof(re));

#if HAS_IMU
    re.type = CHUNK_TYPE_IMU;
    re.rate = config.imu_sample_rate;
    if (fr == FR_OK)
        fr = write_exact(&re, sizeof(re));

    uint8_t imu_meta_count = 0;
    if (imu_frame.available)
        imu_meta_count++;
    if (imu_fork.available)
        imu_meta_count++;
    if (imu_rear.available)
        imu_meta_count++;

    if (fr == FR_OK && imu_meta_count > 0) {
        ch.type = CHUNK_TYPE_IMU_META;
        ch.length = 1 + imu_meta_count * sizeof(struct imu_meta_record);
        if (fr == FR_OK)
            fr = write_exact(&ch, sizeof(ch));
        if (fr == FR_OK)
            fr = write_exact(&imu_meta_count, 1);

        if (fr == FR_OK && imu_frame.available) {
            struct imu_meta_record entry = {0, imu_frame.accel_lsb_per_g, imu_frame.gyro_lsb_per_dps};
            fr = write_exact(&entry, sizeof(entry));
        }
        if (fr == FR_OK && imu_fork.available) {
            struct imu_meta_record entry = {1, imu_fork.accel_lsb_per_g, imu_fork.gyro_lsb_per_dps};
            fr = write_exact(&entry, sizeof(entry));
        }
        if (fr == FR_OK && imu_rear.available) {
            struct imu_meta_record entry = {2, imu_rear.accel_lsb_per_g, imu_rear.gyro_lsb_per_dps};
            fr = write_exact(&entry, sizeof(entry));
        }
    }
#endif

    if (fr == FR_OK)
        fr = f_sync(&recording);

    if (fr != FR_OK) {
        f_close(&recording);
        recording_open = false;
        f_unlink(filename);
        LOG("STORAGE", "Failed to initialize %s: %d\n", filename, fr);
        return storage_backend_status_from_fresult(fr);
    }
    return index;
}

static FRESULT write_chunk(uint8_t type, const void *payload, uint16_t bytes) {
    FSIZE_t start = f_tell(&recording);
    struct chunk_header ch = {type, bytes};
    FRESULT fr = write_exact(&ch, sizeof(ch));
    if (fr == FR_OK && bytes > 0) {
        fr = write_exact(payload, bytes);
    }
    if (fr != FR_OK) {
        FRESULT rollback_fr = f_lseek(&recording, start);
        if (rollback_fr == FR_OK) {
            rollback_fr = f_truncate(&recording);
        }
        if (rollback_fr == FR_OK) {
            rollback_fr = f_sync(&recording);
        }
        return rollback_fr == FR_OK ? fr : rollback_fr;
    }
    return f_sync(&recording);
}

int storage_session_run(void) {
    int index = 0;
    uint32_t command_word;
    uint16_t size;
    FRESULT fr;
    struct travel_record *travel_buffer;
#if HAS_GPS
    struct gps_record *gps_buffer;
#endif
#if HAS_IMU
    struct imu_record *imu_buffer;
#endif

    while (true) {
        command_word = multicore_fifo_pop_blocking();
        if (!core1_fifo_is_family(command_word, CORE1_FIFO_FAMILY_STORAGE_CMD)) {
            return fail_session("Non-storage FIFO word", FR_INT_ERR);
        }

        switch (CORE1_FIFO_ID(command_word)) {
            case STORAGE_CMD_OPEN:
                index = open_datafile();
                storage_send_event(STORAGE_EVENT_OPEN_RESULT);
                multicore_fifo_push_blocking(index);
                multicore_fifo_push_blocking((uintptr_t)travel_databuffer2);
#if HAS_IMU
                multicore_fifo_push_blocking((uintptr_t)imu_databuffer2);
#endif
#if HAS_GPS
                multicore_fifo_push_blocking((uintptr_t)gps_databuffer2);
#endif
                break;
            case STORAGE_CMD_DUMP_TRAVEL:
                size = (uint16_t)multicore_fifo_pop_blocking();
                travel_buffer = (struct travel_record *)((uintptr_t)multicore_fifo_pop_blocking());
                storage_send_event(STORAGE_EVENT_BUFFER_RETURNED);
                multicore_fifo_push_blocking((uintptr_t)travel_buffer);
                fr = write_chunk(CHUNK_TYPE_TRAVEL, travel_buffer, (uint16_t)(size * sizeof(*travel_buffer)));
                if (fr != FR_OK)
                    return fail_session("Write travel chunk", fr);
                break;
#if HAS_GPS
            case STORAGE_CMD_DUMP_GPS:
                size = (uint16_t)multicore_fifo_pop_blocking();
                gps_buffer = (struct gps_record *)((uintptr_t)multicore_fifo_pop_blocking());
                storage_send_event(STORAGE_EVENT_BUFFER_RETURNED);
                multicore_fifo_push_blocking((uintptr_t)gps_buffer);
                fr = write_chunk(CHUNK_TYPE_GPS, gps_buffer, (uint16_t)(size * sizeof(*gps_buffer)));
                if (fr != FR_OK)
                    return fail_session("Write GPS chunk", fr);
                break;
#endif
#if HAS_IMU
            case STORAGE_CMD_DUMP_IMU:
                size = (uint16_t)multicore_fifo_pop_blocking();
                imu_buffer = (struct imu_record *)((uintptr_t)multicore_fifo_pop_blocking());
                storage_send_event(STORAGE_EVENT_BUFFER_RETURNED);
                multicore_fifo_push_blocking((uintptr_t)imu_buffer);
                fr = write_chunk(CHUNK_TYPE_IMU, imu_buffer, (uint16_t)(size * sizeof(*imu_buffer)));
                if (fr != FR_OK)
                    return fail_session("Write IMU chunk", fr);
                break;
#endif
            case STORAGE_CMD_MARKER:
                fr = write_chunk(CHUNK_TYPE_MARKER, NULL, 0);
                if (fr != FR_OK)
                    return fail_session("Write marker chunk", fr);
                break;
            case STORAGE_CMD_FINISH:
                if (recording_open) {
                    fr = f_close(&recording);
                    recording_open = false;
                    if (fr != FR_OK) {
                        LOG("STORAGE", "Close failed: %d\n", fr);
                        return storage_backend_status_from_fresult(fr);
                    }
                }
                return 0;
            default:
                return fail_session("Unknown storage command", FR_INT_ERR);
        }
    }
}
