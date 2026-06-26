#pragma once

#include <gmock/gmock.h>
#include "filesystem_iface.hpp"

class MockFileSystem : public IFileSystem {
public:
    MOCK_METHOD(FileStatus, open, (File* fp, const char* path, const char* mode), (override));
    MOCK_METHOD(FileStatus, close, (File* fp), (override));
    MOCK_METHOD(FileStatus, read, (File* fp, void* buff, uint32_t btr, uint32_t* br), (override));
    MOCK_METHOD(FileStatus, write, (File* fp, const void* buff, uint32_t btw, uint32_t* bw), (override));
    MOCK_METHOD(FileStatus, lseek, (File* fp, uint64_t ofs), (override));
    MOCK_METHOD(FileStatus, truncate, (File* fp), (override));
    MOCK_METHOD(FileStatus, rewind, (File* fp), (override));
    MOCK_METHOD(FileStatus, tell, (File* fp, uint64_t* position), (override));
    MOCK_METHOD(FileStatus, sync, (File* fp), (override));
    MOCK_METHOD(FileStatus, mkdir, (const char* path), (override));
    MOCK_METHOD(FileStatus, unlink, (const char* path), (override));
    MOCK_METHOD(FileStatus, rename, (const char* path_old, const char* path_new), (override));
    MOCK_METHOD(FileStatus, stat, (const char* path, FileInfo* fno), (override));
    MOCK_METHOD(int, putc, (char c, File* fp), (override));
    MOCK_METHOD(int, puts, (const char* str, File* cp), (override));
    MOCK_METHOD(int, printf, (File* fp, const char* str, ...), (override));
    MOCK_METHOD(char*, gets, (char* buff, int len, File* fp), (override));
};