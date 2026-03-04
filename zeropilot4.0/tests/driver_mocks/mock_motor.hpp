#pragma once

#include <gmock/gmock.h>
#include "motor_iface.hpp"

class MockMotorControl : public IMotorControl {
public:
    explicit MockMotorControl(uint8_t id) : servoIdx(id) {}

    MOCK_METHOD(void, set, (uint32_t percent), (override));
    
    uint8_t getServoIdx() const override {
        return servoIdx;
    }

private:
    uint8_t servoIdx;
};
