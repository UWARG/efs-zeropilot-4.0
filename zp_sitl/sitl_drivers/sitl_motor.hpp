#pragma once
#include "motor_iface.hpp"

class SITL_Motor : public IMotorControl {
public:
    void set(uint32_t percent) override {
        currentPercent = percent;
    }
    
    uint32_t get() {
        return currentPercent;
    }

private:
    uint32_t currentPercent = 0;
};
