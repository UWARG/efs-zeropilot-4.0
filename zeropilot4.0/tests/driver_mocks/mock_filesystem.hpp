#pragma once

#include <gmock/gmock.h>
#include "filesystem_iface.hpp"
#include "zp_error.h"

class MockFileSystem : public IFileSystem {
public:
    MOCK_METHOD(ZP_Error, open, (File* fp, const char* path, const char* mode), (override));
    MOCK_METHOD(ZP_Error, write, (ManagerId_e id, File* fp, const void* buff, uint32_t bytesToWrite, uint32_t* bytesWritten, ReqOptions_e options), (override));
    MOCK_METHOD(ZP_Error, writeAndSync, (ManagerId_e id, File* fp, const void* buff, uint32_t bytesToWrite, ReqOptions_e options), (override));
    MOCK_METHOD(ZP_Error, sync, (ManagerId_e id, File* fp, ReqOptions_e options), (override));
    MOCK_METHOD(ZP_Error, mkdir, (const char* path), (override));
    MOCK_METHOD(ZP_Error, stat, (const char* path, FileInfo_t* fno), (override));
    MOCK_METHOD(bool, available, (), (override));
};
