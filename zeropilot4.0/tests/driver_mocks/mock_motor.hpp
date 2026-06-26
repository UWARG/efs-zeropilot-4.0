#pragma once

#include <gmock/gmock.h>
#include "motor_iface.hpp"

class MockMotorControl : public IMotorControl {
public:
    MOCK_METHOD(void, set, (uint32_t percent), (override));
    MOCK_METHOD(void, setArm, (bool arm), (override));
    void init() override {}
};
