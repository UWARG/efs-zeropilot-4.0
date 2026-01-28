#pragma once

#include "textio_iface.hpp"
#include "app_fatfs.h"

class SDIO : public ITextIO {
    public:
        SDIO();
        int mountFile() override;
        int open(char *file, int modes) override;
        int close() override;
        char* read(char *buffer, int bufferSize) override;
        int write(const char *buffer) override;
        int seek(int offset) override;
        uint64_t fsize() override;
        uint64_t tell() override;
        int eof() override;
        bool checkFileExist(char *file) override;

    private:
        FATFS FatFs;
        FIL fil;
};
