#pragma once

#include "filesystem_datatypes.hpp"

class IFileSystem {
    protected:
        IFileSystem() = default;

    public:
        virtual ~IFileSystem() = default;

        // NOTE: for functions like inits that do not happen in a thread, can use the SYNC option to force the operation, functions that don't take IDs are always syncronous

        virtual FileStatus_e open (File* fp, const char* path, const char* mode) = 0;	// Open or create a file
        virtual FileStatus_e write (ManId_e id, File* fp, const void* buff, uint32_t btw, uint32_t* bw, ReqOptions_e options = ReqOptions_e::ASYNC) = 0; // Write data to the file
        virtual FileStatus_e writeAndSync (ManId_e id, File* fp, const void* buff, uint32_t btw, ReqOptions_e options = ReqOptions_e::ASYNC) = 0;	// Write data to the file and sync immediately (for critical logs)
        virtual FileStatus_e sync (ManId_e id, File* fp, ReqOptions_e options = ReqOptions_e::ASYNC) = 0; // Flush cached data of the writing file
        virtual FileStatus_e mkdir (const char* path) = 0; // Create a sub directory 
        virtual FileStatus_e stat (const char* path, FileInfo_t* fno) = 0; // Get file status
        virtual bool available() = 0; // Check if the filesystem is available for use (e.g. SD card is mounted)

        /* TODO: Verify in later PR
        virtual FileStatus_e close (File* fp) = 0; // Close an open file object 
        virtual FileStatus_e read (File* fp, void* buff, uint32_t btr, uint32_t* br) = 0; // Read data from the file 
        virtual FileStatus_e seek_and_write (ManId_e id, File* fp, const void* buff, uint32_t btw, uint64_t ofs, ReqOptions_e options = ReqOptions_e::ASYNC) = 0; // Seek and write data to the file
        virtual FileStatus_e lseek (ManId_e id, File* fp, uint64_t ofs, ReqOptions_e options = ReqOptions_e::ASYNC) = 0; // Move file pointer of the file object
        virtual FileStatus_e tell(ManId_e id, File* fp, uint64_t* position, ReqOptions_e options = ReqOptions_e::ASYNC) = 0; // Get current file pointer position

        virtual int printf (ManId_e id, File* fp, ReqOptions_e options, const char* str, ...) = 0; // Put a formatted string to the file

        virtual PollResult poll(ManId_e id, ReqType_e reqType) = 0;	// Poll for async call result from a manager
        */
};
