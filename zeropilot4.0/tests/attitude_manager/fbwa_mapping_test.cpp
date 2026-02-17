#include <gtest/gtest.h>
#include "fbwa_mapping.hpp"

class FBWAMappingTest : public ::testing::Test {
protected:
    const float CONTROL_PERIOD = 0.01f;
};

TEST_F(FBWAMappingTest, RollPIDControl) {
    FBWAMapping mapper(CONTROL_PERIOD);
    mapper.setRollPIDConstants(1.0f, 0.0f, 0.0f, 0.02f);
    
    RCMotorControlMessage_t input;
    input.roll = 100.0f;
    input.pitch = 50.0f;
    input.yaw = 50.0f;
    input.throttle = 50.0f;
    input.arm = 0.0f;
    input.flapAngle = 0.0f;

    DroneState_t state = DRONE_STATE_DEFAULT;
    
    RCMotorControlMessage_t output = mapper.runControl(input, state);
    
    EXPECT_GT(output.roll, 50.0f);
}

TEST_F(FBWAMappingTest, PitchPIDControl) {
    FBWAMapping mapper(CONTROL_PERIOD);
    mapper.setPitchPIDConstants(1.0f, 0.0f, 0.0f, 0.02f);
    
    RCMotorControlMessage_t input;
    input.roll = 50.0f;
    input.pitch = 100.0f;
    input.yaw = 50.0f;
    input.throttle = 50.0f;
    input.arm = 0.0f;
    input.flapAngle = 0.0f;

    DroneState_t state = DRONE_STATE_DEFAULT;
    
    RCMotorControlMessage_t output = mapper.runControl(input, state);
    
    EXPECT_GT(output.pitch, 50.0f);
}

TEST_F(FBWAMappingTest, YawRudderMixing) {
    FBWAMapping mapper(CONTROL_PERIOD);
    mapper.setRollPIDConstants(1.0f, 0.0f, 0.0f, 0.02f);
    mapper.setYawRudderMixingConstant(0.5f);
    
    RCMotorControlMessage_t input;
    input.roll = 100.0f;
    input.pitch = 50.0f;
    input.yaw = 50.0f;
    input.throttle = 50.0f;
    input.arm = 0.0f;
    input.flapAngle = 0.0f;

    DroneState_t state = DRONE_STATE_DEFAULT;
    
    RCMotorControlMessage_t output = mapper.runControl(input, state);
    
    EXPECT_NE(output.yaw, 50.0f);
}

TEST_F(FBWAMappingTest, YawClamping) {
    FBWAMapping mapper(CONTROL_PERIOD);
    mapper.setRollPIDConstants(1.0f, 0.0f, 0.0f, 0.02f);
    mapper.setYawRudderMixingConstant(0.5f);

    DroneState_t state = DRONE_STATE_DEFAULT;

    // Case 1: Lower Clamping
    RCMotorControlMessage_t inputLower;
    inputLower.roll = 20.0f;
    inputLower.pitch = 50.0f;
    inputLower.yaw = 0.0f;
    inputLower.throttle = 50.0f;
    inputLower.arm = 0.0f;
    inputLower.flapAngle = 0.0f;
    RCMotorControlMessage_t outputLower = mapper.runControl(inputLower, state);
    EXPECT_GE(outputLower.yaw, 0.0f);

    // Resetting the state for the second case to avoid any influence from the first case's control loop state
    mapper.resetControlLoopState();

    // Case 2: Upper Clamping
    RCMotorControlMessage_t inputUpper;
    inputUpper.roll = 80.0f;
    inputUpper.pitch = 50.0f;
    inputUpper.yaw = 100.0f;
    inputUpper.throttle = 50.0f;
    inputUpper.arm = 0.0f;
    inputUpper.flapAngle = 0.0f;
    RCMotorControlMessage_t outputUpper = mapper.runControl(inputUpper, state);
    EXPECT_LE(outputUpper.yaw, 100.0f);
}
