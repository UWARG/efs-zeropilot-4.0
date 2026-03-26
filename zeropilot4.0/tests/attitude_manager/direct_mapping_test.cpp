#include <gtest/gtest.h>
#include "direct_mapping.hpp"

TEST(DirectMappingTest, PassthroughControl) {
    DirectMapping mapper;
    
    RCMotorControlMessage_t input;
    input.roll = 25.0f;
    input.pitch = 75.0f;
    input.yaw = 50.0f;
    input.throttle = 80.0f;
    input.arm = 1.0f;
    input.flapAngle = 30.0f;

    DroneState_t state;
    state.pitch = 0.1f;
    state.roll = 0.2f;
    state.yaw = 0.3f;
    state.altitude = 100.0f;
    state.airspeed = 15.0f;
    
    RCMotorControlMessage_t output = mapper.runControl(input, state);
    
    EXPECT_FLOAT_EQ(output.roll, 25.0f);
    EXPECT_FLOAT_EQ(output.pitch, 75.0f);
    EXPECT_FLOAT_EQ(output.yaw, 50.0f);
    EXPECT_FLOAT_EQ(output.throttle, 80.0f);
    EXPECT_FLOAT_EQ(output.arm, 1.0f);
    EXPECT_FLOAT_EQ(output.flapAngle, 30.0f);
}
