#ifndef _LOG_H
#define _LOG_H

#include <stdio.h>

#ifdef NDEBUG
#define LOG(source, fmt, ...)
#define log_init()
#define log_close()
#else

#ifdef LOG_TO_FILE
void log_write(const char *source, const char *fmt, ...);
void log_init(void);
void log_close(void);
#define LOG(source, fmt, ...) log_write(source, fmt, ##__VA_ARGS__)
#else
#define LOG(source, fmt, ...) printf("[" source "] " fmt, ##__VA_ARGS__)
#define log_init()
#define log_close()
#endif

#endif // NDEBUG

#endif // _LOG_H
