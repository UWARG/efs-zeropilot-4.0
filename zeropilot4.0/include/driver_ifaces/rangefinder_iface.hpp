#pragma once

#include "rangefinder_datatypes.hpp"

class IRangefinder {
    protected:
        IRangefinder() = default;

    public:
        virtual ~IRangefinder() = default;

        virtual int init() = 0;
        virtual RangefinderData_t readData() = 0;
};