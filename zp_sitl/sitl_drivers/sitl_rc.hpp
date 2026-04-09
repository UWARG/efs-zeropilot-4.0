#pragma once
#include "rc_iface.hpp"

class SITL_RC : public IRCReceiver {
private:
    RCControl rcData;
    
public:
    void update_from_commands(float roll, float pitch, float yaw, float throttle, float arm, float flap, float fltmode) {
        rcData.roll = roll;
        rcData.pitch = pitch;
        rcData.yaw = yaw;
        rcData.throttle = throttle;
        rcData.arm = arm;
        rcData.aux2 = flap;
        rcData.fltModeRaw = fltmode;
        rcData.isDataNew = true;
    }
    
    RCControl getRCData() override {
        return rcData;
    }
};
