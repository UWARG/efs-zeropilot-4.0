#pragma once

#include <cstdint>

enum FileStatus {
    FILE_STATUS_OK,
    FILE_STATUS_ERROR
};

typedef struct {
    uint64_t	size;		/* File size */
    uint16_t	date;		/* Modified date */
    uint16_t	time;		/* Modified time */
    uint8_t	    isDir;		/* =1 if dir */
    char	name[255 + 1];	/* Primary file name */
} FileInfo;

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

        virtual FileStatus open (File* fp, const char* path, const char* mode) = 0;			/* Open or create a file */
        virtual FileStatus close (File* fp) = 0;											    /* Close an open file object */
        virtual FileStatus read (File* fp, void* buff, uint32_t btr, uint32_t* br) = 0;			/* Read data from the file */
        virtual FileStatus write (File* fp, const void* buff, uint32_t btw, uint32_t* bw) = 0;	/* Write data to the file */
        virtual FileStatus lseek (File* fp, uint64_t ofs) = 0;								    /* Move file pointer of the file object */
        virtual FileStatus truncate (File* fp) = 0;										        /* Truncate the file */
        virtual FileStatus rewind (File* fp) = 0;												/* Rewind file pointer to the beginning of the file */
        virtual FileStatus tell(File* fp, uint64_t* position) = 0;						        /* Get current file pointer position */
        virtual FileStatus sync (File* fp) = 0;											        /* Flush cached data of the writing file */
        virtual FileStatus mkdir (const char* path) = 0;								        /* Create a sub directory */
        virtual FileStatus unlink (const char* path) = 0;								        /* Delete an existing file or directory */
        virtual FileStatus rename (const char* path_old, const char* path_new) = 0;	            /* Rename/Move a file or directory */
        virtual FileStatus stat (const char* path, FileInfo* fno) = 0;					        /* Get file status */
        
        virtual int putc (char c, File* fp) = 0;										        /* Put a character to the file */
        virtual int puts (const char* str, File* cp) = 0;								        /* Put a string to the file */
        virtual int printf (File* fp, const char* str, ...) = 0;						        /* Put a formatted string to the file */
        virtual char* gets (char* buff, int len, File* fp) = 0;						            /* Get a string from the file */
};
