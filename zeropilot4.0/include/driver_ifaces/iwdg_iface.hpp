#pragma once
#include "zp_error.h"

class IIndependentWatchdog {
    protected:
        IIndependentWatchdog() = default;

    public:
        virtual ~IIndependentWatchdog() = default;

        // reset watchdog timer
        virtual ZP_Error refreshWatchdog() = 0;
};
