#pragma once

#include <gmock/gmock.h>
#include "filesystem_iface.hpp"

class MockFileSystem : public IFileSystem {
public:
    MOCK_METHOD(FileStatus, open, (File* fp, const char* path, const char* mode), (override));
    MOCK_METHOD(FileStatus, write, (ManId id, File* fp, const void* buff, uint32_t btw, uint32_t* bw, ReqOptions options), (override));
    MOCK_METHOD(FileStatus, write_and_sync, (ManId id, File* fp, const void* buff, uint32_t btw, ReqOptions options), (override));
    MOCK_METHOD(FileStatus, sync, (ManId id, File* fp, ReqOptions options), (override));
    MOCK_METHOD(FileStatus, mkdir, (const char* path), (override));
    MOCK_METHOD(FileStatus, stat, (const char* path, FileInfo* fno), (override));
    MOCK_METHOD(bool, available, (), (override));
};
