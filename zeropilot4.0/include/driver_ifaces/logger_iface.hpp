#pragma once

#include "zp_error.h"

class ILogger {
    protected:
        ILogger() = default;
    
    public:
        virtual ~ILogger() = default;

        virtual int log(const char message[100]) = 0;
        virtual int log(const char message[][100], int count) = 0;
};
