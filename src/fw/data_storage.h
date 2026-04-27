#ifndef DATA_STORAGE_H
#define DATA_STORAGE_H

#include <stdbool.h>
#include <stdint.h>

enum { STORAGE_BACKEND_STATUS_FRESULT_BASE = -0x1000 };

static inline int32_t storage_backend_status_from_fresult(int32_t fresult) {
    return STORAGE_BACKEND_STATUS_FRESULT_BASE - fresult;
}

static inline bool storage_backend_status_is_fresult(int32_t status) {
    return status <= STORAGE_BACKEND_STATUS_FRESULT_BASE - 1 && status >= STORAGE_BACKEND_STATUS_FRESULT_BASE - 255;
}

static inline int32_t storage_backend_status_to_fresult(int32_t status) {
    return STORAGE_BACKEND_STATUS_FRESULT_BASE - status;
}

int setup_storage(void);
int storage_session_run(void);

#endif // DATA_STORAGE_H