#pragma once

#include "rangefinder_datatypes.hpp"
#include "zp_error.h"

class IRangefinder {
    protected:
        IRangefinder() = default;

    public:
        virtual ~IRangefinder() = default;

        virtual ZP_Error init() = 0;
        virtual ZP_Error readData(RangefinderData_t &data) = 0;
};
