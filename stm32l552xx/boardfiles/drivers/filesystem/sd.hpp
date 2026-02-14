#pragma once

#include "filesystem_iface.hpp"
#include "app_fatfs.h"

class SDFileSystem : public IFileSystem {
    private:
        FATFS fsObj;
        bool mounted;
        
        // Helper to convert FatFs result to FileStatus
        FileStatus fresultToStatus(FRESULT res);
        
        // Helper to convert mode string to FatFs mode flags
        BYTE modeStringToFatfsFlags(const char* mode);

    public:
        SDFileSystem();
        ~SDFileSystem() override;
        
        FileStatus init();
        
        // IFileSystem implementation
        FileStatus open(File* fp, const char* path, const char* mode) override;
        FileStatus close(File* fp) override;
        FileStatus read(File* fp, void* buff, uint32_t btr, uint32_t* br) override;
        FileStatus write(File* fp, const void* buff, uint32_t btw, uint32_t* bw) override;
        FileStatus lseek(File* fp, uint64_t ofs) override;
        FileStatus truncate(File* fp) override;
        FileStatus rewind(File* fp) override;
        FileStatus tell(File* fp, uint64_t* position) override;
        FileStatus sync(File* fp) override;
        FileStatus mkdir(const char* path) override;
        FileStatus unlink(const char* path) override;
        FileStatus rename(const char* path_old, const char* path_new) override;
        FileStatus stat(const char* path, FileInfo* fno) override;
        
        int putc(char c, File* fp) override;
        int puts(const char* str, File* cp) override;
        int printf(File* fp, const char* str, ...) override;
        char* gets(char* buff, int len, File* fp) override;
};
