#pragma once

#include "filesystem_iface.hpp"
#include "zp_error.h"
#include "sd_msgs.hpp"
#include "queue.hpp"
#include "fatfs.h"

class SDFileSystem : public IFileSystem {
    public:
        SDFileSystem(MessageQueue<SdReqMsg> *reqQueue, MessageQueue<SdReqBuf> *bufQueue, IMessageQueue<PollResult> *respQueues[static_cast<size_t>(ManagerId_e::NUM_MANAGERS)]);
        ~SDFileSystem() override;
        
        ZP_Error init();

        // Helper to convert FatFs result to ZP_Error
        static ZP_Error fresultToZpError(FRESULT res);
        
        // IFileSystem implementation
        ZP_Error open (File* fp, const char* path, const char* mode) override;
        ZP_Error mkdir (const char* path) override;
        ZP_Error write (ManagerId_e id, File* fp, const void* buff, uint32_t bytesToWrite, uint32_t* bytesWritten, ReqOptions_e options = ReqOptions_e::ASYNC) override;
        ZP_Error writeAndSync (ManagerId_e id, File* fp, const void* buff, uint32_t bytesToWrite, ReqOptions_e options = ReqOptions_e::ASYNC) override;
        ZP_Error sync (ManagerId_e id, File* fp, ReqOptions_e options = ReqOptions_e::ASYNC) override;
        ZP_Error stat (const char* path, FileInfo_t* fno) override;
        bool available();									       

        /* TODO: Verify in later PR
        ZP_Error close (File* fp) override;
        ZP_Error read (File* fp, void* buff, uint32_t btr, uint32_t* br) override;
        ZP_Error seek_and_write (ManagerId_e id, File* fp, const void* buff, uint32_t bytesToWrite, uint64_t ofs, ReqOptions_e options = ReqOptions_e::ASYNC) override;
        ZP_Error lseek (ManagerId_e id, File* fp, uint64_t ofs, ReqOptions_e options = ReqOptions_e::ASYNC) override;
        ZP_Error tell(ManagerId_e id, File* fp, uint64_t* position, ReqOptions_e options = ReqOptions_e::ASYNC) override;
        int printf (ManagerId_e id, File* fp, ReqOptions_e options, const char* str, ...) override;
        PollResult poll(ManagerId_e id, ReqType_e reqType) override;		
        */

    private:
        FATFS fsObj;
        bool mounted;
        
        // Helper to convert mode string to FatFs mode flags
        uint8_t modeStringToFatfsFlags(const char* mode);

        MessageQueue<SdReqMsg> *requestQueue;
        MessageQueue<SdReqBuf> *bufferQueue;
        IMessageQueue<PollResult> **responseQueues; // Array of response queues for each manager ID
};
