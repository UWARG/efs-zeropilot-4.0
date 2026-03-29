#pragma once

#include <cstdint>

#define MAX_RW_BUFFER_SIZE 128  // Maximum buffer size for read/write operations, can be adjusted as needed

enum class ManId : uint8_t {
    SYSTEM = 0,
    ATTITUDE,
    TELEMERTRY,
    COUNT
};

enum FileStatus {
    FILE_STATUS_OK,
    FILE_STATUS_ERROR,
    FILE_STATUS_REQUEST_MADE,  // For async operations where the request has been made but not completed yet
    FILE_STATUS_REQUEST_FAILED, // For async operations where the request was made but failed to start
    FILE_STATUS_NOT_DONE,  // For poll function if async operation is not completed yet
    FILE_STATUS_UNKNOWN // For unrecognized operations or errors
};

enum class ReqType : uint8_t {
    WRITE,
    LSEEK,
    TELL,
    SYNC,
    WRITE_SYNC, // For write operations that require immediate flush to storage like critical logs
    WRITE_SEEK, // For write operations that require seeking to a specific position before writing
};

enum class ReqOptions : uint8_t {
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
} FileInfo;

struct PollResult {
    ReqType type;
    FileStatus status;
    union {
        uint64_t position;           // For tell operation
        uint32_t bytes_transferred;  // For write operations
    } data;
};

struct alignas(4) File {
    char _storage[256];  // Safe margin: FIL (~200 bytes) + FILE (~150 bytes) + padding
};

// this assumes we're using UTF-8, never UTF-16

// API is similar to https://elm-chan.org/fsw/ff/

class IFileSystem {
    protected:
        IFileSystem() = default;

    public:
        virtual ~IFileSystem() = default;

// fsize is uint64_t
// UINT is uint32_t

// since we have truncate exposed as an api, that means by default we open files as rb+ mode in POSIX)
// NOTE: for functions like inits that do not happen in a thread, can use the SYNC option to force the operation, functions that don't take IDs are always syncronous

        virtual FileStatus open (File* fp, const char* path, const char* mode) = 0;			/* Open or create a file */
        virtual FileStatus close (File* fp) = 0;											    /* Close an open file object */
        virtual FileStatus read (File* fp, void* buff, uint32_t btr, uint32_t* br) = 0;			/* Read data from the file */
        virtual FileStatus write (ManId id, File* fp, const void* buff, uint32_t btw, uint32_t* bw, ReqOptions options = ReqOptions::ASYNC) = 0;	/* Write data to the file */
        virtual FileStatus seek_and_write (ManId id, File* fp, const void* buff, uint32_t btw, uint64_t ofs, ReqOptions options = ReqOptions::ASYNC) = 0;	/* Seek and write data to the file */
        virtual FileStatus write_and_sync (ManId id, File* fp, const void* buff, uint32_t btw, ReqOptions options = ReqOptions::ASYNC) = 0;	/* Write data to the file and sync immediately (for critical logs) */
        virtual FileStatus lseek (ManId id, File* fp, uint64_t ofs, ReqOptions options = ReqOptions::ASYNC) = 0;								    /* Move file pointer of the file object */
        virtual FileStatus tell(ManId id, File* fp, uint64_t* position, ReqOptions options = ReqOptions::ASYNC) = 0;						        /* Get current file pointer position */
        virtual FileStatus sync (ManId id, File* fp, ReqOptions options = ReqOptions::ASYNC) = 0;											        /* Flush cached data of the writing file */
        virtual FileStatus mkdir (const char* path) = 0;								        /* Create a sub directory */
        virtual FileStatus stat (const char* path, FileInfo* fno) = 0;					        /* Get file status */

        virtual int printf (ManId id, File* fp, ReqOptions options, const char* str, ...) = 0;						        /* Put a formatted string to the file */

        virtual PollResult poll(ManId id, ReqType reqType) = 0;										        /* Poll for async call result from a manager */
        virtual bool available() = 0;                                                    /* Check if the filesystem is available for use (e.g. SD card is mounted) */
};
