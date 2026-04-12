#include "sst.h"

#include "ff.h"

// Duration is derived from the total travel record count divided by the travel
// sample rate. This requires scanning all chunk headers in the file.
struct sst_file_info sst_get_file_info(const char *path) {
    struct sst_file_info info = {0};
    uint br;
    FIL f;

    FRESULT fr = f_open(&f, path, FA_OPEN_EXISTING | FA_READ);
    if (!(fr == FR_OK || fr == FR_EXIST)) {
        return info;
    }

    struct sst_header header;
    fr = f_read(&f, &header, sizeof(header), &br);
    if (fr != FR_OK || br != sizeof(header)) {
        f_close(&f);
        return info;
    }

    info.version = header.version;
    info.timestamp = header.timestamp;

    uint16_t travel_rate = 0;
    uint32_t travel_records = 0;
    struct chunk_header ch;

    while (f_read(&f, &ch, sizeof(ch), &br) == FR_OK && br == sizeof(ch)) {
        if (ch.type == CHUNK_TYPE_RATES) {
            uint16_t remaining = ch.length;
            while (remaining >= sizeof(struct samplerate_record)) {
                struct samplerate_record sr;
                if (f_read(&f, &sr, sizeof(sr), &br) != FR_OK || br != sizeof(sr)) {
                    goto done;
                }
                if (sr.type == CHUNK_TYPE_TRAVEL) {
                    travel_rate = sr.rate;
                }
                remaining -= sizeof(sr);
            }
        } else {
            if (ch.type == CHUNK_TYPE_TRAVEL) {
                travel_records += ch.length / sizeof(struct travel_record);
            }
            if (f_lseek(&f, f_tell(&f) + ch.length) != FR_OK) {
                break;
            }
        }
    }

done:
    f_close(&f);

    if (travel_rate > 0) {
        info.duration_ms = (uint32_t)((uint64_t)travel_records * 1000 / travel_rate);
    }

    return info;
}
