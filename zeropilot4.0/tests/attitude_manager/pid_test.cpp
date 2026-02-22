#include <gtest/gtest.h>
#include "pid.hpp"

class PIDTest : public ::testing::Test {
protected:
    const float KP = 1.0f;
    const float KI = 0.1f;
    const float KD = 0.05f;
    const float TAU = 0.02f;
    const float DT = 0.01f;
    const float OUTPUT_MIN = -50.0f;
    const float OUTPUT_MAX = 50.0f;
    const float INTEGRAL_MIN = -25.0f;
    const float INTEGRAL_MAX = 25.0f;
};

TEST_F(PIDTest, ProportionalControl) {
    PID pid(KP, 0.0f, 0.0f, TAU, OUTPUT_MIN, OUTPUT_MAX, INTEGRAL_MIN, INTEGRAL_MAX, DT);
    pid.pidInitState();
    
    float output = pid.pidOutput(10.0f, 0.0f);
    EXPECT_FLOAT_EQ(output, KP * 10.0f);
}

TEST_F(PIDTest, IntegralAccumulation) {
    PID pid(0.0f, KI, 0.0f, TAU, OUTPUT_MIN, OUTPUT_MAX, INTEGRAL_MIN, INTEGRAL_MAX, DT);
    pid.pidInitState();
    
    float error = 10.0f;
    pid.pidOutput(error, 0.0f);
    float output = pid.pidOutput(error, 0.0f);
    
    EXPECT_GT(output, 0.0f);
}

TEST_F(PIDTest, IntegralWindupClamping) {
    PID pid(0.0f, 10.0f, 0.0f, TAU, OUTPUT_MIN, OUTPUT_MAX, INTEGRAL_MIN, INTEGRAL_MAX, DT);
    pid.pidInitState();
    
    for (int i = 0; i < 100; i++) {
        pid.pidOutput(100.0f, 0.0f);
    }
    
    float output = pid.pidOutput(100.0f, 0.0f);
    EXPECT_LE(output, INTEGRAL_MAX);
    EXPECT_GE(output, INTEGRAL_MIN);
}

TEST_F(PIDTest, OutputClamping) {
    PID pid(100.0f, 0.0f, 0.0f, TAU, OUTPUT_MIN, OUTPUT_MAX, INTEGRAL_MIN, INTEGRAL_MAX, DT);
    pid.pidInitState();
    
    float output = pid.pidOutput(100.0f, 0.0f);
    EXPECT_EQ(output, OUTPUT_MAX);
    
    output = pid.pidOutput(-100.0f, 0.0f);
    EXPECT_EQ(output, OUTPUT_MIN);
}

TEST_F(PIDTest, DerivativeResponse) {
    PID pid(0.0f, 0.0f, KD, TAU, OUTPUT_MIN, OUTPUT_MAX, INTEGRAL_MIN, INTEGRAL_MAX, DT);
    pid.pidInitState();
    
    pid.pidOutput(0.0f, 0.0f);
    float output = pid.pidOutput(0.0f, 10.0f);
    
    EXPECT_LT(output, 0.0f);
}

TEST_F(PIDTest, StateReset) {
    PID pid(KP, KI, KD, TAU, OUTPUT_MIN, OUTPUT_MAX, INTEGRAL_MIN, INTEGRAL_MAX, DT);
    pid.pidInitState();
    
    for (int i = 0; i < 10; i++) {
        pid.pidOutput(10.0f, 0.0f);
    }
    
    pid.pidInitState();
    float output = pid.pidOutput(10.0f, 0.0f);
    
    EXPECT_NEAR(output, 10.0f, 0.1f);
}
