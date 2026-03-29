#pragma once

#include "filesystem_iface.hpp"
#include "sd_fatfs_msgs.hpp"
#include "queue.hpp"
#include "app_fatfs.h"

class SDFileSystem : public IFileSystem {
    private:
        FATFS fsObj;
        bool mounted;
        
        
        // Helper to convert FatFs result to FileStatus
        FileStatus fresultToStatus(FRESULT res);
        
        // Helper to convert mode string to FatFs mode flags
        BYTE modeStringToFatfsFlags(const char* mode);

        MessageQueue<FatFSReqMsg> *requestQueue;
        MessageQueue<FatFSReqBuff> *bufferQueue;
        MessageQueue<FatFSRespMsg> **responseQueues; // Array of response queues for each manager ID

    public:
        SDFileSystem(MessageQueue<FatFSReqMsg> *reqQueue, MessageQueue<FatFSReqBuff> *buffQueue, MessageQueue<FatFSRespMsg> *respQueues[ManId::COUNT]);
        ~SDFileSystem() override;
        
        FileStatus init();
        
        // IFileSystem implementation
        FileStatus open (ManId id, File* fp, const char* path, const char* mode) override;
        FileStatus close (ManId id, File* fp) override;
        FileStatus read (ManId id, File* fp, void* buff, uint32_t btr, uint32_t* br) override;
        FileStatus write (ManId id, File* fp, const void* buff, uint32_t btw, uint32_t* bw, ReqOptions options = ReqOptions::ASYNC) override;
        FileStatus seek_and_write (ManId id, File* fp, const void* buff, uint32_t btw, uint32_t* bw, uint64_t ofs) override;
        FileStatus write_and_sync (ManId id, File* fp, const void* buff, uint32_t btw, uint32_t* bw) override;
        FileStatus lseek (ManId id, File* fp, uint64_t ofs, ReqOptions options = ReqOptions::ASYNC) override;
        FileStatus tell(ManId id, File* fp, uint64_t* position, ReqOptions options = ReqOptions::ASYNC) override;
        FileStatus sync (ManId id, File* fp, ReqOptions options = ReqOptions::ASYNC) override;
        FileStatus mkdir (ManId id, const char* path, bool forceSync = false) override;
        FileStatus stat (ManId id, const char* path, FileInfo* fno, bool forceSync = false) override;					        
        int printf (ManId id, File* fp, const char* str, ...) override;
        PollResult poll(ManId id, ReqType reqType) override;		
        bool available();									       
};
