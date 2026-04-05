#if defined(LOG_TO_FILE) && !defined(NDEBUG)

#include "log.h"

#include "../ntp/ntp.h"
#include "ff.h"

#include <stdarg.h>
#include <stdio.h>
#include <time.h>

static FIL log_file;
static bool log_file_open = false;

void log_init(void) {
    time_t now = rtc_timestamp();
    struct tm *t = gmtime(&now);

    f_mkdir("logs");

    char filename[28];
    snprintf(filename, sizeof(filename), "logs/LOG_%02d%02d%02d%02d%02d.TXT", t->tm_year % 100, t->tm_mon + 1,
             t->tm_mday, t->tm_hour, t->tm_min);

    FRESULT fr = f_open(&log_file, filename, FA_CREATE_ALWAYS | FA_WRITE);
    if (fr == FR_OK) {
        log_file_open = true;
    }
}

void log_close(void) {
    if (log_file_open) {
        f_close(&log_file);
        log_file_open = false;
    }
}

void log_write(const char *source, const char *fmt, ...) {
    char buf[256];
    int n = snprintf(buf, sizeof(buf), "[%s] ", source);

    va_list args;
    va_start(args, fmt);
    n += vsnprintf(buf + n, sizeof(buf) - n, fmt, args);
    va_end(args);

    printf("%s", buf);

    if (log_file_open) {
        f_write(&log_file, buf, n, NULL);
        f_sync(&log_file);
    }
}

#endif // LOG_TO_FILE && !NDEBUG
