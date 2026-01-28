#pragma once

#include <cstdint>
class ITextIO {
    protected:
        ITextIO() = default;

    public:
        ~ITextIO() = default;

        virtual int mountFile() = 0;
        virtual int open(char *file, int modes) = 0;
        virtual int close() = 0;
        virtual char* read(char *buffer, int bufferSize) = 0;
        virtual int write(const char *buffer) = 0;
        virtual int seek(int offset) = 0;
        virtual uint64_t fsize() = 0;
        virtual uint64_t tell() = 0;
        virtual int eof() = 0;
        virtual bool checkFileExist(char *file) = 0;
};
