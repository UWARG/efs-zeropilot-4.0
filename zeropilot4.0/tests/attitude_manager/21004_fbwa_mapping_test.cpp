#include <gtest/gtest.h>
#include "fbwa_mapping.hpp"

class FBWAMappingTest : public ::testing::Test {
protected:
    const float CONTROL_PERIOD = 0.01f;
};

TEST_F(FBWAMappingTest, RollPIDControl) {
    FBWAMapping mapper(CONTROL_PERIOD);
    mapper.setRollPIDConstants(1.0f, 0.0f, 0.0f, 0.02f);
    
    RCMotorControlMessage_t input = {100.0f, 50.0f, 50.0f, 50.0f, 0.0f, 0.0f};
    DroneState_t state = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    
    RCMotorControlMessage_t output = mapper.runControl(input, state);
    
    EXPECT_GT(output.roll, 50.0f);
}

TEST_F(FBWAMappingTest, PitchPIDControl) {
    FBWAMapping mapper(CONTROL_PERIOD);
    mapper.setPitchPIDConstants(1.0f, 0.0f, 0.0f, 0.02f);
    
    RCMotorControlMessage_t input = {50.0f, 100.0f, 50.0f, 50.0f, 0.0f, 0.0f};
    DroneState_t state = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    
    RCMotorControlMessage_t output = mapper.runControl(input, state);
    
    EXPECT_GT(output.pitch, 50.0f);
}

TEST_F(FBWAMappingTest, YawRudderMixing) {
    FBWAMapping mapper(CONTROL_PERIOD);
    mapper.setRollPIDConstants(1.0f, 0.0f, 0.0f, 0.02f);
    mapper.setYawRudderMixingConstant(0.5f);
    
    RCMotorControlMessage_t input = {100.0f, 50.0f, 50.0f, 50.0f, 0.0f, 0.0f};
    DroneState_t state = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    
    RCMotorControlMessage_t output = mapper.runControl(input, state);
    
    EXPECT_NE(output.yaw, 50.0f);
}

TEST_F(FBWAMappingTest, YawClampingLower) {
    FBWAMapping mapper(CONTROL_PERIOD);
    mapper.setRollPIDConstants(1.0f, 0.0f, 0.0f, 0.02f);
    mapper.setYawRudderMixingConstant(-2.0f);
    
    RCMotorControlMessage_t input = {100.0f, 50.0f, 0.0f, 50.0f, 0.0f, 0.0f};
    DroneState_t state = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    
    RCMotorControlMessage_t output = mapper.runControl(input, state);
    
    EXPECT_GE(output.yaw, 0.0f);
}

TEST_F(FBWAMappingTest, YawClampingUpper) {
    FBWAMapping mapper(CONTROL_PERIOD);
    mapper.setRollPIDConstants(1.0f, 0.0f, 0.0f, 0.02f);
    mapper.setYawRudderMixingConstant(2.0f);
    
    RCMotorControlMessage_t input = {100.0f, 50.0f, 100.0f, 50.0f, 0.0f, 0.0f};
    DroneState_t state = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    
    RCMotorControlMessage_t output = mapper.runControl(input, state);
    
    EXPECT_LE(output.yaw, 100.0f);
}
