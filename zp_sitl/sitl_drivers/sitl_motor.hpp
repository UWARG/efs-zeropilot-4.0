#pragma once
#include "motor_iface.hpp"

class SITL_Motor : public IMotorControl {
public:
    SITL_Motor() = default;

    void set(uint32_t percent) override {
        currentPercent = percent;
    }

    void init() override {}

    uint32_t get() {
        return currentPercent;
    }

    void setArm(bool arm) override { 
        armFlag = arm;
    }

private:
    uint32_t currentPercent = 0;
};
