#ifndef CORE1_IPC_H
#define CORE1_IPC_H

#include <stdbool.h>
#include <stdint.h>

#define CORE1_FIFO_FAMILY_MASK           0xFF000000u
#define CORE1_FIFO_ID_MASK               0x0000FFFFu
#define CORE1_FIFO_FAMILY_DISPATCH_CMD   0x01000000u
#define CORE1_FIFO_FAMILY_DISPATCH_EVENT 0x02000000u
#define CORE1_FIFO_FAMILY_STORAGE_CMD    0x03000000u
#define CORE1_FIFO_FAMILY_STORAGE_EVENT  0x04000000u

#define CORE1_FIFO_WORD(family, id) ((uint32_t)((family) | ((uint32_t)(id) & CORE1_FIFO_ID_MASK)))
#define CORE1_FIFO_FAMILY(word)     ((uint32_t)(word) & CORE1_FIFO_FAMILY_MASK)
#define CORE1_FIFO_ID(word)         ((uint32_t)(word) & CORE1_FIFO_ID_MASK)

enum core1_mode {
    CORE1_MODE_IDLE = 0,
    CORE1_MODE_STORAGE,
    CORE1_MODE_TCP_SERVER,
    CORE1_MODE_STOPPING,
    CORE1_MODE_ERROR,
};

enum core1_dispatch_command {
    CORE1_DISPATCH_CMD_ENTER_STORAGE = 1,
    CORE1_DISPATCH_CMD_ENTER_TCP_SERVER = 2,
};

enum core1_dispatch_event {
    CORE1_DISPATCH_EVENT_STORAGE_READY = 1,
    CORE1_DISPATCH_EVENT_TCP_SERVER_READY = 2,
    CORE1_DISPATCH_EVENT_BACKEND_COMPLETE = 3,
    CORE1_DISPATCH_EVENT_BACKEND_ERROR = 4,
};

enum storage_session_command {
    STORAGE_CMD_OPEN = 1,
    STORAGE_CMD_DUMP_TRAVEL = 2,
    STORAGE_CMD_DUMP_GPS = 3,
    STORAGE_CMD_DUMP_IMU = 4,
    STORAGE_CMD_FINISH = 5,
    STORAGE_CMD_MARKER = 6,
};

enum storage_session_event {
    STORAGE_EVENT_OPEN_RESULT = 1,
    STORAGE_EVENT_BUFFER_RETURNED = 2,
};

static inline bool core1_fifo_is_family(uint32_t word, uint32_t family) { return CORE1_FIFO_FAMILY(word) == family; }

#endif // CORE1_IPC_H