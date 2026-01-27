#pragma once

#include "logger_iface.hpp"
#include "app_fatfs.h"
#include "logger_config.h"
#include "error.h"

class Logger : public ILogger {
    private:
        FATFS FatFs;
        FIL fil;
        char file[100];

    public:
        Logger() = default;

        /**
         * @brief logs a single message to the SD Card
         * @param message: data to be written
         * @retval ZP_ERROR_e: Operation result
         */
        ZP_ERROR_e log(const char message[100]);

        /**
         * @brief logs multiple messages to the SD card
         * @param messages: data to be written
         * @retval ZP_ERROR_e: Operation result
         */
        ZP_ERROR_e log(const char messages[][100], int count);

        /**
         * @brief mounts SD card and selects file to write to, call before starting kernel
         * @retval ZP_ERROR_e: Operation result
         */
        ZP_ERROR_e init();

};
