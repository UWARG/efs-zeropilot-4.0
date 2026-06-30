#pragma once
#include "filesystem_datatypes.hpp"

class IFileSystemBackend {
  public:
    virtual ~IFileSystemBackend() = default;

    virtual FileStatus_e writeFile(File* fp, const void* buf, uint32_t len, uint32_t* written) = 0;
    virtual FileStatus_e syncFile(File* fp) = 0;
    virtual FileStatus_e seekFile(File* fp, uint64_t ofs) = 0;
    virtual uint64_t tellFile(File* fp) = 0;
};
