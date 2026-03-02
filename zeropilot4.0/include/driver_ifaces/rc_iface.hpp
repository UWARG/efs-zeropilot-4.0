#pragma once

#include "rc_datatypes.hpp"

class IRCReceiver { 
    protected:
        IRCReceiver() = default; 

    public:
        virtual ~IRCReceiver() = default;

        // get RCControl data that is parsed from sbus
        virtual RCControl getRCData() = 0;

        // convert RCControl data to SBUS/CRSF/MAVLink
        virtual void forcePushMAVLinkRC(RCControl rcData) = 0;
};