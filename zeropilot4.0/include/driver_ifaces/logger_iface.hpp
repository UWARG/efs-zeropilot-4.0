#pragma once

#include "error.h"

class ILogger {
    protected:
        ILogger() = default;

    public:
        virtual ~ILogger() = default;

        virtual ZP_ERROR_e log(const char message[100]) = 0;
        virtual ZP_ERROR_e log(const char message[][100], int count) = 0;
};
