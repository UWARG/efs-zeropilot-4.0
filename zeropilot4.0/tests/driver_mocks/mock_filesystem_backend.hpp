#pragma once

#include <gmock/gmock.h>
#include "filesystem_backend_iface.hpp"

class MockFileSystemBackend : public IFileSystemBackend {
public:
    MOCK_METHOD(FileStatus_e, writeFile, (File* fp, const void* buf, uint32_t len, uint32_t* written), (override));
    MOCK_METHOD(FileStatus_e, syncFile, (File* fp), (override));
    MOCK_METHOD(FileStatus_e, seekFile, (File* fp, uint64_t ofs), (override));
    MOCK_METHOD(uint64_t, tellFile, (File* fp), (override));
};
