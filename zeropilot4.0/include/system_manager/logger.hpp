#pragma once

#include "logger_params.h"
#include "textio_iface.hpp"
#include "systemutils_iface.hpp"

#define FA_READ             0x01
#define FA_WRITE            0x02
#define FA_OPEN_EXISTING    0x00
#define FA_CREATE_NEW       0x04
#define FA_CREATE_ALWAYS    0x08
#define FA_OPEN_ALWAYS      0x10
#define FA_OPEN_APPEND      0x30

class Logger {
    private:
        char logFile[100];
        ITextIO *textIO;
        ISystemUtils *sysUtils;

    public:
        Logger(ITextIO *textIO, ISystemUtils *sysUtils);

        /**
         * @brief logs a single message to the SD Card
         * @param message data to be written
         * @retval DRESULT: Operation result
         */
        int log(const char message[100]);

        /**
         * @brief logs multiple messages to the SD card
         * @param messages data to be written
         * @param count number of messages to write
         * @retval DRESULT: Operation result
         */
        int log(const char messages[][100], int count);

        /**
         * @brief mounts SD card and selects file to write to, call before starting kernel
         */
        int init();

};