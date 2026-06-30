#pragma once

#include <gmock/gmock.h>
#include "filesystem_iface.hpp"

class MockFileSystem : public IFileSystem {
public:
    MOCK_METHOD(FileStatus_e, open, (File* fp, const char* path, const char* mode), (override));
    MOCK_METHOD(FileStatus_e, write, (ManId_e id, File* fp, const void* buff, uint32_t btw, uint32_t* bw, ReqOptions_e options), (override));
    MOCK_METHOD(FileStatus_e, writeAndSync, (ManId_e id, File* fp, const void* buff, uint32_t btw, ReqOptions_e options), (override));
    MOCK_METHOD(FileStatus_e, sync, (ManId_e id, File* fp, ReqOptions_e options), (override));
    MOCK_METHOD(FileStatus_e, mkdir, (const char* path), (override));
    MOCK_METHOD(FileStatus_e, stat, (const char* path, FileInfo_t* fno), (override));
    MOCK_METHOD(bool, available, (), (override));
};
