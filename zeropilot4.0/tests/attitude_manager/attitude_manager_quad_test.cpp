#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "attitude_manager.hpp"
#include "zp_params.hpp"
#include "mock_systemutils.hpp"
#include "mock_gps.hpp"
#include "mock_imu.hpp"
#include "mock_queue.hpp"
#include "mock_motor.hpp"
#include "mock_fft.hpp"
#include "mock_rangefinder.hpp"

using ::testing::_;
using ::testing::Return;
using ::testing::DoAll;
using ::testing::SetArgPointee;
using ::testing::AtLeast;
using ::testing::Invoke;
using ::testing::NiceMock;
using ::testing::AnyNumber;
using ::testing::Gt;

class AttitudeManagerQuadTest : public ::testing::Test {
protected:
    int AM_RC_FAILSAFE_ITERATIONS;

    NiceMock<MockSystemUtils> mockSystemUtils;
    NiceMock<MockFFT> mockFFT;
    NiceMock<MockGPS> mockGPS;
    NiceMock<MockIMU> mockIMU;
    NiceMock<MockRangefinder> mockRangefinder;
    NiceMock<MockMessageQueue<RCMotorControlMessage_t>> mockAMQueue;
    NiceMock<MockMessageQueue<TMMessage_t>> mockTMQueue;
    NiceMock<MockMessageQueue<char[100]>> mockLogQueue;

    NiceMock<MockMotorControl> motor1;
    NiceMock<MockMotorControl> motor2;
    NiceMock<MockMotorControl> motor3;
    NiceMock<MockMotorControl> motor4;

    MotorInstance_t motorInstances[4] = {
        {&motor1},
        {&motor2},
        {&motor3},
        {&motor4}
    }; // Remaining fields overwritten by AMParamSetup::loadAllParams() from ZP_PARAM

    MotorGroupInstance_t motorGroup{motorInstances, 4};

    void SetUp() override {
        ZP_PARAM::init();

        // Test uses 4 motors: MOTOR_1, MOTOR_2, MOTOR_3, MOTOR_4
        // No need to override other params such as TRIM, MIN, MAX, REVERSED, bc they are not used in quad
        ZP_PARAM::setParamById("SERVO1_FUNCTION", static_cast<float>(MotorFunction_e::MOTOR_1));
        ZP_PARAM::setParamById("SERVO2_FUNCTION", static_cast<float>(MotorFunction_e::MOTOR_2));
        ZP_PARAM::setParamById("SERVO3_FUNCTION", static_cast<float>(MotorFunction_e::MOTOR_3));
        ZP_PARAM::setParamById("SERVO4_FUNCTION", static_cast<float>(MotorFunction_e::MOTOR_4));

        AM_RC_FAILSAFE_ITERATIONS =
            static_cast<int>(((ZP_PARAM::get(ZP_PARAM_ID::RC_FS_TIMEOUT)) * 1000) / AM_UPDATE_LOOP_DELAY_MS) + 5;

        ON_CALL(mockSystemUtils, getCurrentTimestampMs()).WillByDefault(Return(1000));
        ON_CALL(mockIMU, readRawData()).WillByDefault(Return(RawImuBatch_t{}));      // Empty batch, count 0
        ON_CALL(mockIMU, scaleIMUData(_)).WillByDefault(Return(ScaledImuBatch_t{})); // Empty batch, count 0
        ON_CALL(mockGPS, readData()).WillByDefault(Return(GpsData_t{}));
        ON_CALL(mockAMQueue, count()).WillByDefault(Return(0));
        ON_CALL(mockTMQueue, push(_)).WillByDefault(Return(0));
        ON_CALL(mockFFT, init(_)).WillByDefault(Return(true));
        ON_CALL(mockRangefinder, init()).WillByDefault(Return(0));
    }
};

TEST_F(AttitudeManagerQuadTest, AllMotorsDisarmedOnStartup) {
    EXPECT_CALL(motor1, set(0)).Times(AnyNumber());
    EXPECT_CALL(motor2, set(0)).Times(AnyNumber());
    EXPECT_CALL(motor3, set(0)).Times(AnyNumber());
    EXPECT_CALL(motor4, set(0)).Times(AnyNumber());

    // any nonzero set() call is a test failure
    EXPECT_CALL(motor1, set(Gt(0))).Times(0);
    EXPECT_CALL(motor2, set(Gt(0))).Times(0);
    EXPECT_CALL(motor3, set(Gt(0))).Times(0);
    EXPECT_CALL(motor4, set(Gt(0))).Times(0);

        AttitudeManager am(&mockSystemUtils, &mockGPS, &mockIMU, &mockFFT, & mockRangefinder, &mockAMQueue, &mockTMQueue, &mockLogQueue, &motorGroup);

    am.amUpdate();
}

TEST_F(AttitudeManagerQuadTest, MotorOutputTest) {
    RCMotorControlMessage_t rcMsg;
    rcMsg.roll = 60.0f;
    rcMsg.pitch = 70.0f;
    rcMsg.yaw = 55.0f;
    rcMsg.throttle = 80.0f;
    rcMsg.arm = true;
    rcMsg.flightMode = FlightMode_e::ACRO;

    EXPECT_CALL(mockAMQueue, count()).WillOnce(Return(1));
    EXPECT_CALL(mockAMQueue, get(_)).WillOnce(DoAll(SetArgPointee<0>(rcMsg), Return(0)));

    EXPECT_CALL(motor1, set(_)).Times(AtLeast(1));
    EXPECT_CALL(motor2, set(_)).Times(AtLeast(1));
    EXPECT_CALL(motor3, set(_)).Times(AtLeast(1));
    EXPECT_CALL(motor4, set(_)).Times(AtLeast(1));

        AttitudeManager am(&mockSystemUtils, &mockGPS, &mockIMU, &mockFFT, & mockRangefinder, &mockAMQueue, &mockTMQueue, &mockLogQueue, &motorGroup);

    am.amUpdate();
}

TEST_F(AttitudeManagerQuadTest, DisarmThrottleZero) {
    RCMotorControlMessage_t rcMsg;
    rcMsg.roll = 50.0f;
    rcMsg.pitch = 50.0f;
    rcMsg.yaw = 50.0f;
    rcMsg.throttle = 80.0f;
    rcMsg.arm = false;
    rcMsg.flightMode = FlightMode_e::ACRO;

    EXPECT_CALL(mockAMQueue, count()).WillOnce(Return(1));
    EXPECT_CALL(mockAMQueue, get(_)).WillOnce(DoAll(SetArgPointee<0>(rcMsg), Return(0)));

    EXPECT_CALL(motor1, set(0)).Times(AtLeast(1));
    EXPECT_CALL(motor2, set(0)).Times(AtLeast(1));
    EXPECT_CALL(motor3, set(0)).Times(AtLeast(1));
    EXPECT_CALL(motor4, set(0)).Times(AtLeast(1));

        AttitudeManager am(&mockSystemUtils, &mockGPS, &mockIMU, &mockFFT, & mockRangefinder, &mockAMQueue, &mockTMQueue, &mockLogQueue, &motorGroup);

    am.amUpdate();
}

TEST_F(AttitudeManagerQuadTest, FailsafeTriggered) {
    EXPECT_CALL(mockAMQueue, count()).WillRepeatedly(Return(0));
    EXPECT_CALL(mockLogQueue, push(_)).Times(1);

    EXPECT_CALL(motor1, set(0)).Times(AtLeast(1));
    EXPECT_CALL(motor2, set(0)).Times(AtLeast(1));
    EXPECT_CALL(motor3, set(0)).Times(AtLeast(1));
    EXPECT_CALL(motor4, set(0)).Times(AtLeast(1));

        AttitudeManager am(&mockSystemUtils, &mockGPS, &mockIMU, &mockFFT, & mockRangefinder, &mockAMQueue, &mockTMQueue, &mockLogQueue, &motorGroup);

    for (int i = 0; i < AM_RC_FAILSAFE_ITERATIONS; i++) {
        am.amUpdate();
    }
}

TEST_F(AttitudeManagerQuadTest, FailsafeRecovery) {
    RCMotorControlMessage_t rcMsg;
    rcMsg.roll = 50.0f;
    rcMsg.pitch = 50.0f;
    rcMsg.yaw = 50.0f;
    rcMsg.throttle = 50.0f;
    rcMsg.arm = true;
    rcMsg.flightMode = FlightMode_e::ACRO;

    testing::Sequence seq;
    EXPECT_CALL(mockAMQueue, count())
        .Times(AM_RC_FAILSAFE_ITERATIONS)
        .InSequence(seq)
        .WillRepeatedly(Return(0));
    EXPECT_CALL(mockAMQueue, count())
        .InSequence(seq)
        .WillOnce(Return(1));
    EXPECT_CALL(mockAMQueue, get(_))
        .InSequence(seq)
        .WillOnce(DoAll(SetArgPointee<0>(rcMsg), Return(0)));

    EXPECT_CALL(mockLogQueue, push(_)).Times(2);

        AttitudeManager am(&mockSystemUtils, &mockGPS, &mockIMU, &mockFFT, & mockRangefinder, &mockAMQueue, &mockTMQueue, &mockLogQueue, &motorGroup);

    for (int i = 0; i < AM_RC_FAILSAFE_ITERATIONS; i++) {
        am.amUpdate();
    }

    am.amUpdate();
}

TEST_F(AttitudeManagerQuadTest, MotorClampingUpper) {
    RCMotorControlMessage_t rcMsg;
    rcMsg.roll = 50.0f;
    rcMsg.pitch = 50.0f;
    rcMsg.yaw = 50.0f;
    rcMsg.throttle = 150.0f;
    rcMsg.arm = true;
    rcMsg.flightMode = FlightMode_e::ACRO;

    EXPECT_CALL(mockAMQueue, count()).WillOnce(Return(1));
    EXPECT_CALL(mockAMQueue, get(_)).WillOnce(DoAll(SetArgPointee<0>(rcMsg), Return(0)));

    EXPECT_CALL(motor1, set(100));
    EXPECT_CALL(motor2, set(100));
    EXPECT_CALL(motor3, set(100));
    EXPECT_CALL(motor4, set(100));

        AttitudeManager am(&mockSystemUtils, &mockGPS, &mockIMU, &mockFFT, & mockRangefinder, &mockAMQueue, &mockTMQueue, &mockLogQueue, &motorGroup);

    am.amUpdate();
}
