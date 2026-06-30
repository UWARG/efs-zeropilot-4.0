#pragma once

#include "filesystem_datatypes.hpp"

struct ExMemReqMsg {
    ManId_e id;
    ReqType_e type;
    File* fp = nullptr;
    uint32_t totalSize; // For write operations
    uint64_t offset;     // For lseek operation
    uint8_t modeFlags = 0;
    bool sendResp = true;
};

struct ExMemReqBuff {
    ManId_e id;
    ReqType_e type;
    char buff[MAX_RW_BUFFER_SIZE];
    uint32_t size;
};
