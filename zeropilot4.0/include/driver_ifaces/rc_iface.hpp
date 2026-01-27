#pragma once

#include "rc_datatypes.hpp"
#include "error.h"

class IRCReceiver {
    protected:
        IRCReceiver() = default;

    public:
        virtual ~IRCReceiver() = default;

        // get RCControl data that is parsed from sbus
        virtual ZP_ERROR_e getRCData(RCControl *data) = 0;
};