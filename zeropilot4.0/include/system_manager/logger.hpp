#pragma once

#include "logger_params.hpp"
#include "textio_iface.hpp"
#include "systemutils_iface.hpp"

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
