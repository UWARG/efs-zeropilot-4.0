#pragma once

#include "zp_error.h"
#include "filesystem_datatypes.hpp"

class IFileSystem {
    protected:
        IFileSystem() = default;

    public:
        virtual ~IFileSystem() = default;

        // NOTE: for functions like inits that do not happen in a thread, can use the SYNC option to force the operation, functions that don't take IDs are always synchronous

        virtual ZP_Error open (
            File* fp, 
            const char* path, 
            const char* mode) = 0; // Open or create a file
        virtual ZP_Error write (
            ManagerId_e id, 
            File* fp, 
            const void* buff, 
            uint32_t bytesToWrite, 
            uint32_t* bytesWritten, 
            ReqOptions_e options = ReqOptions_e::ASYNC) = 0; // Write data to the file
        virtual ZP_Error writeAndSync (
            ManagerId_e id, 
            File* fp, 
            const void* buff, 
            uint32_t bytesToWrite, 
            ReqOptions_e options = ReqOptions_e::ASYNC) = 0; // Write data to the file and sync immediately (for critical logs)
        virtual ZP_Error sync (
            ManagerId_e id, 
            File* fp, 
            ReqOptions_e options = ReqOptions_e::ASYNC) = 0; // Flush cached data of the writing file
        virtual ZP_Error mkdir (const char* path) = 0; // Create a sub directory
        virtual ZP_Error stat (const char* path, FileInfo_t* fno) = 0; // Get file status
        virtual bool available() = 0; // Check if the filesystem is available for use (e.g. SD card is mounted)

        /* TODO: Verify in later PR
        virtual ZP_Error close (File* fp) = 0; // Close an open file object
        virtual ZP_Error read (File* fp, void* buff, uint32_t btr, uint32_t* br) = 0; // Read data from the file
        virtual ZP_Error seek_and_write (ManagerId_e id, File* fp, const void* buff, uint32_t bytesToWrite, uint64_t ofs, ReqOptions_e options = ReqOptions_e::ASYNC) = 0; // Seek and write data to the file
        virtual ZP_Error lseek (ManagerId_e id, File* fp, uint64_t ofs, ReqOptions_e options = ReqOptions_e::ASYNC) = 0; // Move file pointer of the file object
        virtual ZP_Error tell(ManagerId_e id, File* fp, uint64_t* position, ReqOptions_e options = ReqOptions_e::ASYNC) = 0; // Get current file pointer position

        virtual int printf (ManagerId_e id, File* fp, ReqOptions_e options, const char* str, ...) = 0; // Put a formatted string to the file

        virtual PollResult poll(ManagerId_e id, ReqType_e reqType) = 0;	// Poll for async call result from a manager
        */
};
