#include "data_storage.h"
#include "data_acquisition.h"

#include "core1_ipc.h"

#include "sensor_setup.h"
#include "sst.h"

#include "../ntp/ntp.h"
#include "../util/log.h"

#include "ff.h"
#include "pico/multicore.h"
#include "pico/platform.h"
#include "pico/unique_id.h"

#include <stdint.h>
#include <stdio.h>

static FIL recording;

static void storage_send_event(enum storage_session_event event_id) {
    multicore_fifo_push_blocking(CORE1_FIFO_WORD(CORE1_FIFO_FAMILY_STORAGE_EVENT, event_id));
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
        return fr;
    }

    struct sst_header h = {"SST", 4, 0, rtc_timestamp()};
    f_write(&recording, &h, sizeof(struct sst_header), NULL);

#if HAS_IMU
    struct chunk_header ch = {CHUNK_TYPE_RATES, 2 * sizeof(struct samplerate_record)};
#else
    struct chunk_header ch = {CHUNK_TYPE_RATES, 1 * sizeof(struct samplerate_record)};
#endif
    f_write(&recording, &ch, sizeof(struct chunk_header), NULL);
    struct samplerate_record re = {CHUNK_TYPE_TRAVEL, TRAVEL_SAMPLE_RATE};
    f_write(&recording, &re, sizeof(struct samplerate_record), NULL);

#if HAS_IMU
    re.type = CHUNK_TYPE_IMU;
    re.rate = IMU_SAMPLE_RATE;
    f_write(&recording, &re, sizeof(struct samplerate_record), NULL);

    uint8_t imu_meta_count = 0;
    if (imu_frame.available)
        imu_meta_count++;
    if (imu_fork.available)
        imu_meta_count++;
    if (imu_rear.available)
        imu_meta_count++;

    if (imu_meta_count > 0) {
        ch.type = CHUNK_TYPE_IMU_META;
        ch.length = 1 + imu_meta_count * sizeof(struct imu_meta_record);
        f_write(&recording, &ch, sizeof(struct chunk_header), NULL);
        f_write(&recording, &imu_meta_count, 1, NULL);

        if (imu_frame.available) {
            struct imu_meta_record entry = {0, imu_frame.accel_lsb_per_g, imu_frame.gyro_lsb_per_dps};
            f_write(&recording, &entry, sizeof(struct imu_meta_record), NULL);
        }
        if (imu_fork.available) {
            struct imu_meta_record entry = {1, imu_fork.accel_lsb_per_g, imu_fork.gyro_lsb_per_dps};
            f_write(&recording, &entry, sizeof(struct imu_meta_record), NULL);
        }
        if (imu_rear.available) {
            struct imu_meta_record entry = {2, imu_rear.accel_lsb_per_g, imu_rear.gyro_lsb_per_dps};
            f_write(&recording, &entry, sizeof(struct imu_meta_record), NULL);
        }
    }
#endif

    return index;
}

static void write_travel_chunk(uint16_t size, struct travel_record *buffer) {
    struct chunk_header ch;
    ch.type = CHUNK_TYPE_TRAVEL;
    ch.length = size * sizeof(struct travel_record);
    f_write(&recording, &ch, sizeof(struct chunk_header), NULL);
    f_write(&recording, buffer, ch.length, NULL);
    f_sync(&recording);
}

#if HAS_GPS
static void write_gps_chunk(uint16_t size, struct gps_record *buffer) {
    struct chunk_header ch;
    ch.type = CHUNK_TYPE_GPS;
    ch.length = size * sizeof(struct gps_record);
    f_write(&recording, &ch, sizeof(struct chunk_header), NULL);
    f_write(&recording, buffer, ch.length, NULL);
    f_sync(&recording);
}
#endif

#if HAS_IMU
static void write_imu_chunk(uint16_t size, struct imu_record *buffer) {
    struct chunk_header ch;
    ch.type = CHUNK_TYPE_IMU;
    ch.length = size * sizeof(struct imu_record);
    f_write(&recording, &ch, sizeof(struct chunk_header), NULL);
    f_write(&recording, buffer, ch.length, NULL);
    f_sync(&recording);
}
#endif

int storage_session_run(void) {
    int index;
    uint32_t command_word;
    uint16_t size;
    struct travel_record *travel_buffer;
#if HAS_GPS
    struct gps_record *gps_buffer;
#endif
#if HAS_IMU
    struct imu_record *imu_buffer;
#endif
    struct chunk_header ch;

    while (true) {
        command_word = multicore_fifo_pop_blocking();
        if (!core1_fifo_is_family(command_word, CORE1_FIFO_FAMILY_STORAGE_CMD)) {
            return PICO_ERROR_GENERIC;
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
                write_travel_chunk(size, travel_buffer);
                break;
#if HAS_GPS
            case STORAGE_CMD_DUMP_GPS:
                size = (uint16_t)multicore_fifo_pop_blocking();
                gps_buffer = (struct gps_record *)((uintptr_t)multicore_fifo_pop_blocking());
                storage_send_event(STORAGE_EVENT_BUFFER_RETURNED);
                multicore_fifo_push_blocking((uintptr_t)gps_buffer);
                write_gps_chunk(size, gps_buffer);
                break;
#endif
#if HAS_IMU
            case STORAGE_CMD_DUMP_IMU:
                size = (uint16_t)multicore_fifo_pop_blocking();
                imu_buffer = (struct imu_record *)((uintptr_t)multicore_fifo_pop_blocking());
                storage_send_event(STORAGE_EVENT_BUFFER_RETURNED);
                multicore_fifo_push_blocking((uintptr_t)imu_buffer);
                write_imu_chunk(size, imu_buffer);
                break;
#endif
            case STORAGE_CMD_MARKER:
                ch.type = CHUNK_TYPE_MARKER;
                ch.length = 0;
                f_write(&recording, &ch, sizeof(struct chunk_header), NULL);
                f_sync(&recording);
                break;
            case STORAGE_CMD_FINISH:
                return f_close(&recording) == FR_OK ? 0 : PICO_ERROR_GENERIC;
            default:
                return PICO_ERROR_GENERIC;
        }
    }
}