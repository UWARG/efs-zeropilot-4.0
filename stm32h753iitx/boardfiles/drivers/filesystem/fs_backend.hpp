#pragma once
#include "filesystem_backend_iface.hpp"

class FatFsBackend : public IFileSystemBackend {
  public:
    FileStatus_e writeFile(File* fp, const void* buf, uint32_t len, uint32_t* written) override;
    FileStatus_e syncFile(File* fp) override;
    FileStatus_e seekFile(File* fp, uint64_t ofs) override;
    uint64_t   tellFile(File* fp) override;
};
