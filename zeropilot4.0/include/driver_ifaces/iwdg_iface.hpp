#pragma once

#include "error.h"

class IIndependentWatchdog {
    protected:
        IIndependentWatchdog() = default;

    public:
        virtual ~IIndependentWatchdog() = default;

        // reset watchdog timer
        virtual ZP_ERROR_e refreshWatchdog() = 0;
};
