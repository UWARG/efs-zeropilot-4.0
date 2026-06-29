#pragma once

#include "filesystem_datatypes.hpp"

struct ExMemReqMsg {
    ManId id;
    ReqType type;
    File* fp = nullptr;
    uint32_t total_size; // For write operations
    uint64_t offset;     // For lseek operation
    uint8_t mode_flags = 0;
    bool sendResp = true;
};

struct ExMemReqBuff {
    ManId id;
    ReqType type;
    char buff[MAX_RW_BUFFER_SIZE];
    uint32_t size;
};
