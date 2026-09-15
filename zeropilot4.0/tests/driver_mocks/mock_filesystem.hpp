#pragma once

#include <gmock/gmock.h>
#include "filesystem_iface.hpp"

class MockFileSystem : public IFileSystem {
public:
    MOCK_METHOD(FileStatus_e, open, (File* fp, const char* path, const char* mode), (override));
    MOCK_METHOD(FileStatus_e, write, (ManagerId_e id, File* fp, const void* buff, uint32_t bytesToWrite, uint32_t* bytesWritten, ReqOptions_e options), (override));
    MOCK_METHOD(FileStatus_e, writeAndSync, (ManagerId_e id, File* fp, const void* buff, uint32_t bytesToWrite, ReqOptions_e options), (override));
    MOCK_METHOD(FileStatus_e, sync, (ManagerId_e id, File* fp, ReqOptions_e options), (override));
    MOCK_METHOD(FileStatus_e, mkdir, (const char* path), (override));
    MOCK_METHOD(FileStatus_e, stat, (const char* path, FileInfo_t* fno), (override));
    MOCK_METHOD(bool, available, (), (override));
};
