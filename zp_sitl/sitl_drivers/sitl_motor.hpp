#pragma once
#include "motor_iface.hpp"

class SITL_Motor : public IMotorControl {
public:
    SITL_Motor(uint8_t servoIdx) : servoIdx(servoIdx) {}

    void set(uint32_t percent) override {
        currentPercent = percent;
    }

    uint8_t getServoIdx() const override {
        return servoIdx;
    }
    
    uint32_t get() {
        return currentPercent;
    }

private:
    uint32_t currentPercent = 0;
    uint8_t servoIdx;
};
