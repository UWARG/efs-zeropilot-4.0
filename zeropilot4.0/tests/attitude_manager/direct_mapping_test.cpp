#include <gtest/gtest.h>
#include "direct_mapping.hpp"

TEST(DirectMappingTest, PassthroughControl) {
    DirectMapping mapper;
    
    RCMotorControlMessage_t input = {25.0f, 75.0f, 50.0f, 80.0f, 1.0f, 30.0f};
    DroneState_t state = {0.1f, 0.2f, 0.3f, 100.0f, 15.0f};
    
    RCMotorControlMessage_t output = mapper.runControl(input, state);
    
    EXPECT_FLOAT_EQ(output.roll, 25.0f);
    EXPECT_FLOAT_EQ(output.pitch, 75.0f);
    EXPECT_FLOAT_EQ(output.yaw, 50.0f);
    EXPECT_FLOAT_EQ(output.throttle, 80.0f);
    EXPECT_FLOAT_EQ(output.arm, 1.0f);
    EXPECT_FLOAT_EQ(output.flapAngle, 30.0f);
}
