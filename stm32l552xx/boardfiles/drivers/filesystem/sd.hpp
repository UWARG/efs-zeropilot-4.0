#pragma once

#include "filesystem_iface.hpp"
#include "exmem_msgs.hpp"
#include "queue.hpp"
#include "app_fatfs.h"

class SDFileSystem : public IFileSystem {
    private:
        FATFS fsObj;
        bool mounted;
        
        // Helper to convert mode string to FatFs mode flags
        BYTE modeStringToFatfsFlags(const char* mode);

        MessageQueue<ExMemReqMsg> *requestQueue;
        MessageQueue<ExMemReqBuff> *bufferQueue;
        IMessageQueue<PollResult> **responseQueues; // Array of response queues for each manager ID

    public:
        SDFileSystem(MessageQueue<ExMemReqMsg> *reqQueue, MessageQueue<ExMemReqBuff> *buffQueue, IMessageQueue<PollResult> *respQueues[static_cast<size_t>(ManId_e::COUNT)]);
        ~SDFileSystem() override;
        
        FileStatus_e init();

        // Helper to convert FatFs result to FileStatus_e
        static FileStatus_e fresultToStatus(FRESULT res);
        
        // IFileSystem implementation
        FileStatus_e open (File* fp, const char* path, const char* mode) override;
        FileStatus_e mkdir (const char* path) override;
        FileStatus_e write (ManId_e id, File* fp, const void* buff, uint32_t btw, uint32_t* bw, ReqOptions_e options = ReqOptions_e::ASYNC) override;
        FileStatus_e writeAndSync (ManId_e id, File* fp, const void* buff, uint32_t btw, ReqOptions_e options = ReqOptions_e::ASYNC) override;
        FileStatus_e sync (ManId_e id, File* fp, ReqOptions_e options = ReqOptions_e::ASYNC) override;
        FileStatus_e stat (const char* path, FileInfo_t* fno) override;					        
        bool available();

        /* TODO: Verify in later PR
        FileStatus_e close (File* fp) override;
        FileStatus_e read (File* fp, void* buff, uint32_t btr, uint32_t* br) override;
        FileStatus_e seek_and_write (ManId_e id, File* fp, const void* buff, uint32_t btw, uint64_t ofs, ReqOptions_e options = ReqOptions_e::ASYNC) override;
        FileStatus_e lseek (ManId_e id, File* fp, uint64_t ofs, ReqOptions_e options = ReqOptions_e::ASYNC) override;
        FileStatus_e tell(ManId_e id, File* fp, uint64_t* position, ReqOptions_e options = ReqOptions_e::ASYNC) override;
        int printf (ManId_e id, File* fp, ReqOptions_e options, const char* str, ...) override;
        PollResult poll(ManId_e id, ReqType_e reqType) override;		
        */
};
