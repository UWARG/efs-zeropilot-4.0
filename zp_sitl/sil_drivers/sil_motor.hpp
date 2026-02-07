#pragma once
#include "motor_iface.hpp"

class SIL_Motor : public IMotorControl {
public:
    uint32_t currentPercent = 0;
    
    void set(uint32_t percent) override {
        currentPercent = percent;
    }
    
    uint32_t get() {
        return currentPercent;
    }
};
