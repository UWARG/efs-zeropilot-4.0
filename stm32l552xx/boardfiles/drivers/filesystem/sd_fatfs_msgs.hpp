#pragma once

#include "filesystem_iface.hpp"
#include "app_fatfs.h"

struct FatFSReqMsg {
    ManId id;
    ReqType type;
    FIL* fp = nullptr;
    uint32_t total_size; // For read/write operations
    uint64_t offset;     // For lseek operation
    BYTE mode_flags = 0;
    bool sendResp = true;
};

struct FatFSReqBuff {
    ManId id;
    ReqType type;
    char buff[MAX_RW_BUFFER_SIZE];
    uint32_t size;
};
