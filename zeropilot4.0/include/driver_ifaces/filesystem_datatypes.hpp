#pragma once

#include <cstdint>
#include "zp_error.h"

#define MAX_RW_BUFFER_SIZE 256 // Maximum buffer size for read/write operations, can be adjusted as needed
#define MAX_FILE_NAME_LENGTH 255 // FAT long-filename limit

enum class ManagerId_e : uint8_t {
    SYSTEM = 0,
    ATTITUDE,
    TELEMRETRY,
    NUM_MANAGERS
};

enum class ReqType_e : uint8_t {
    WRITE,
    LSEEK,
    TELL,
    SYNC,
    WRITE_SYNC, // For write operations that require immediate flush to storage like critical logs
    WRITE_SEEK, // For write operations that require seeking to a specific position before writing
};

enum class ReqOptions_e : uint8_t {
    ASYNC = 0,
    ASYNC_NO_RESP, // For async operations where the manager should not send a response (used for fire-and-forget operations like logging)
    SYNC
};

typedef struct {
    uint64_t size; /* File size */
    uint16_t date; /* Modified date */
    uint16_t time; /* Modified time */
    uint8_t isDir; /* =1 if dir */
    char name[MAX_FILE_NAME_LENGTH + 1]; /* Primary file name */
} FileInfo_t;

struct PollResult {
    ReqType_e type;
    ZP_Error status;
    union {
        uint64_t position; // For tell operation
        uint32_t bytesTransferred;  // For write operations
    } data;
};

struct File {
    alignas(4) char storage[256];  // Safe margin: FIL (~200 bytes) + FILE (~150 bytes) + padding
};
