#pragma once
#include "killswitch_datatypes.hpp"

class IKillSwitch {
protected:
    IKillSwitch() = default;
public:
    virtual ~IKillSwitch() = default;

    // latch 
    virtual bool isPressed() const = 0;

    //clear latch
    virtual void forceKill() = 0;
    virtual void clearKill() = 0;

    virtual void buzzerOn() = 0;
    virtual void buzzerOff() = 0;
};
