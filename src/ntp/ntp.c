#include "ntp.h"
#include "lwip/apps/sntp.h"
#include "pico/aon_timer.h"
#include "pico/platform.h"
#include "pico/time.h"
#include <string.h>

#include "../rtc/ds3231.h"

extern struct ds3231 rtc;

static volatile uint64_t start_time_us = 0;
static volatile bool ntp_done = false;

static int64_t days_from_civil(int year, unsigned month, unsigned day) {
    year -= month <= 2u;
    int era = (year >= 0 ? year : year - 399) / 400;
    unsigned year_of_era = (unsigned)(year - era * 400);
    unsigned day_of_year = (153u * (month + (month > 2u ? (unsigned)-3 : 9u)) + 2u) / 5u + day - 1u;
    unsigned day_of_era = year_of_era * 365u + year_of_era / 4u - year_of_era / 100u + day_of_year;
    return (int64_t)era * 146097 + (int64_t)day_of_era - 719468;
}

static time_t utc_epoch_from_tm(const struct tm *tm_utc) {
    int year = tm_utc->tm_year + 1900;
    unsigned month = (unsigned)tm_utc->tm_mon + 1u;
    unsigned day = (unsigned)tm_utc->tm_mday;
    int64_t days = days_from_civil(year, month, day);
    int64_t seconds = (((days * 24) + tm_utc->tm_hour) * 60 + tm_utc->tm_min) * 60 + tm_utc->tm_sec;
    return (time_t)seconds;
}

time_t rtc_timestamp() {
#if PICO_RP2040
    // RP2040: calendar methods are native (direct RTC hardware access), no timezone conversion
    struct tm tm_now;
    if (!aon_timer_get_time_calendar(&tm_now)) {
        return 0;
    }

    return utc_epoch_from_tm(&tm_now);
#else
    // RP2350: use linear time methods (native to Powman Timer), no timezone conversion
    struct timespec ts;
    aon_timer_get_time(&ts);
    return ts.tv_sec; // Already UTC timestamp
#endif
}

bool sync_rtc_to_ntp() {
    ntp_done = false;
    sntp_init();

    absolute_time_t timeout_time = make_timeout_time_ms(NTP_TIMEOUT_TIME);
    while (!ntp_done && absolute_time_diff_us(get_absolute_time(), timeout_time) > 0) { tight_loop_contents(); }

    sntp_stop();

    return ntp_done;
}

void setup_ntp(const char *server) {
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, server);
    if (aon_timer_is_running()) {
        start_time_us = rtc_timestamp() * 1000000;
    } else {
        start_time_us = 0;
    }
}

uint64_t get_system_time_us() { return start_time_us + time_us_64(); }

bool set_system_time_utc(time_t epoch_seconds, uint32_t micros) {
    struct tm tm_utc;

    if (micros >= 1000000u || gmtime_r(&epoch_seconds, &tm_utc) == NULL) {
        return false;
    }

#if PICO_RP2040
    aon_timer_set_time_calendar(&tm_utc);
#else
    struct timespec ts = {.tv_sec = epoch_seconds, .tv_nsec = (long)micros * 1000L};
    aon_timer_set_time(&ts);
#endif

    // Always update the external DS3231 RTC with UTC time
    ds3231_set_datetime(&rtc, &tm_utc);

    start_time_us = (((uint64_t)epoch_seconds * 1000000u) + micros) - time_us_64();
    ntp_done = true;
    return true;
}

void set_system_time_us(uint32_t sec, uint32_t us) { (void)set_system_time_utc((time_t)sec, us); }
