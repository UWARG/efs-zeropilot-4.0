#pragma once
#include "rc_iface.hpp"

class SITL_RC : public IRCReceiver {
private:
    RCControl rcData;
    
public:
    void update_from_commands(float roll, float pitch, float yaw, float throttle, float arm) {
        rcData.roll = roll;
        rcData.pitch = pitch;
        rcData.yaw = yaw;
        rcData.throttle = throttle;
        rcData.arm = arm;
        rcData.isDataNew = true;
    }
    
    RCControl getRCData() override {
        return rcData;
    }
};
