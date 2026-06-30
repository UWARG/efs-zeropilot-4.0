#pragma once

#include <cstdint>

#define MAX_RW_BUFFER_SIZE 256  // Maximum buffer size for read/write operations, can be adjusted as needed

enum class ManId_e : uint8_t {
    SYSTEM = 0,
    ATTITUDE,
    TELEMERTRY,
    COUNT
};

enum FileStatus_e {
    FILE_STATUS_OK,
    FILE_STATUS_ERROR,
    FILE_STATUS_REQUEST_MADE,  // For async operations where the request has been made but not completed yet
    FILE_STATUS_REQUEST_FAILED, // For async operations where the request was made but failed to start
    FILE_STATUS_NOT_DONE,  // For poll function if async operation is not completed yet
    FILE_STATUS_UNKNOWN // For unrecognized operations or errors
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
    uint64_t	size;		/* File size */
    uint16_t	date;		/* Modified date */
    uint16_t	time;		/* Modified time */
    uint8_t	    isDir;		/* =1 if dir */
    char	name[255 + 1];	/* Primary file name */
} FileInfo_t;

struct PollResult {
    ReqType_e type;
    FileStatus_e status;
    union {
        uint64_t position;           // For tell operation
        uint32_t bytesTransferred;  // For write operations
    } data;
};

struct File {
    alignas(4) char storage[256];  // Safe margin: FIL (~200 bytes) + FILE (~150 bytes) + padding
};
