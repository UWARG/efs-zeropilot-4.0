#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "attitude_manager.hpp"
#include "zp_params.hpp"
#include "zp_bit.hpp"
#include "mock_systemutils.hpp"
#include "mock_gps.hpp"
#include "mock_imu.hpp"
#include "mock_queue.hpp"
#include "mock_motor.hpp"
#include "mock_fft.hpp"
#include "mock_rangefinder.hpp"
#include "mock_mathutils.hpp"
#include "mock_barometer.hpp"

using ::testing::_;
using ::testing::Return;
using ::testing::SetArgReferee;
using ::testing::DoAll;
using ::testing::SetArgPointee;
using ::testing::AtLeast;
using ::testing::Invoke;
using ::testing::NiceMock;
using ::testing::AnyNumber;
using ::testing::Ge;
using ::testing::Gt;

class AttitudeManagerQuadTest : public ::testing::Test {
protected:
    uint32_t nowMs = 1000;
    uint32_t rcFailMs = 0;

    NiceMock<MockSystemUtils> mockSystemUtils;
    NiceMock<MockFFT> mockFFT;
    NiceMock<MockMathUtils> mockMathUtils;
    NiceMock<MockGPS> mockGPS;
    NiceMock<MockIMU> mockIMU;
    NiceMock<MockRangefinder> mockRangefinder;
    NiceMock<MockBarometer> mockBarometer;
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

        float rcFsTimeout = 0.0f;
        (void)ZP_PARAM::get(ZP_PARAM_ID::RC_FS_TIMEOUT, rcFsTimeout);
        rcFailMs = static_cast<uint32_t>(rcFsTimeout * 1000.0f);

        ON_CALL(mockSystemUtils, getCurrentTimestampMs()).WillByDefault(Invoke([this]() { return nowMs; }));

        (void)ZP_BIT::init(&mockSystemUtils);
        (void)ZP_BIT::setPersistence(ZP_BIT_ID::RC_DATA_VALID, rcFailMs, AM_UPDATE_LOOP_DELAY_MS * 3);
        ON_CALL(mockIMU, readRawData(_)).WillByDefault(DoAll(SetArgReferee<0>(RawImuBatch_t{}), Return(ZP_ERROR_OK)));      // Empty batch, count 0
        ON_CALL(mockIMU, scaleIMUData(_, _)).WillByDefault(DoAll(SetArgReferee<1>(ScaledImuBatch_t{}), Return(ZP_ERROR_OK))); // Empty batch, count 0
        ON_CALL(mockGPS, readData(_)).WillByDefault(DoAll(SetArgReferee<0>(GpsData_t{}), Return(ZP_ERROR_OK)));
        ON_CALL(mockAMQueue, count(_)).WillByDefault(DoAll(SetArgReferee<0>(0), Return(ZP_ERROR_OK)));
        ON_CALL(mockTMQueue, push(_)).WillByDefault(Return(ZP_ERROR_OK));
        ON_CALL(mockFFT, init(_)).WillByDefault(Return(ZP_ERROR_OK));
        ON_CALL(mockRangefinder, init()).WillByDefault(Return(ZP_ERROR_OK));
    }

    void failRcBit() {
        (void)ZP_BIT::report(ZP_BIT_ID::RC_DATA_VALID, ZP_ERROR_OK);
        (void)ZP_BIT::report(ZP_BIT_ID::RC_DATA_VALID, ZP_ERROR_NOT_READY);
        nowMs += rcFailMs + 1;
        (void)ZP_BIT::report(ZP_BIT_ID::RC_DATA_VALID, ZP_ERROR_NOT_READY);
    }

    void clearRcBit() {
        (void)ZP_BIT::report(ZP_BIT_ID::RC_DATA_VALID, ZP_ERROR_OK);
        nowMs += rcFailMs + 1;
        (void)ZP_BIT::report(ZP_BIT_ID::RC_DATA_VALID, ZP_ERROR_OK);
        (void)ZP_BIT::clearLatched();
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

    AttitudeManager am(&mockSystemUtils, &mockMathUtils, &mockGPS, &mockIMU, &mockFFT, &mockRangefinder, &mockBarometer, &mockAMQueue, &mockTMQueue, &mockLogQueue, &motorGroup);

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

    EXPECT_CALL(mockAMQueue, count(_)).WillOnce(DoAll(SetArgReferee<0>(1), Return(ZP_ERROR_OK)));
    EXPECT_CALL(mockAMQueue, get(_)).WillOnce(DoAll(SetArgPointee<0>(rcMsg), Return(ZP_ERROR_OK)));

    EXPECT_CALL(motor1, set(_)).Times(AtLeast(1));
    EXPECT_CALL(motor2, set(_)).Times(AtLeast(1));
    EXPECT_CALL(motor3, set(_)).Times(AtLeast(1));
    EXPECT_CALL(motor4, set(_)).Times(AtLeast(1));

    AttitudeManager am(&mockSystemUtils, &mockMathUtils, &mockGPS, &mockIMU, &mockFFT, &mockRangefinder, &mockBarometer, &mockAMQueue, &mockTMQueue, &mockLogQueue, &motorGroup);

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

    EXPECT_CALL(mockAMQueue, count(_)).WillOnce(DoAll(SetArgReferee<0>(1), Return(ZP_ERROR_OK)));
    EXPECT_CALL(mockAMQueue, get(_)).WillOnce(DoAll(SetArgPointee<0>(rcMsg), Return(ZP_ERROR_OK)));

    EXPECT_CALL(motor1, set(0)).Times(AtLeast(1));
    EXPECT_CALL(motor2, set(0)).Times(AtLeast(1));
    EXPECT_CALL(motor3, set(0)).Times(AtLeast(1));
    EXPECT_CALL(motor4, set(0)).Times(AtLeast(1));

    AttitudeManager am(&mockSystemUtils, &mockMathUtils, &mockGPS, &mockIMU, &mockFFT, &mockRangefinder, &mockBarometer, &mockAMQueue, &mockTMQueue, &mockLogQueue, &motorGroup);

    am.amUpdate();
}

TEST_F(AttitudeManagerQuadTest, FailsafeTriggered) {
    EXPECT_CALL(mockAMQueue, count(_)).WillRepeatedly(DoAll(SetArgReferee<0>(0), Return(ZP_ERROR_OK)));
    EXPECT_CALL(mockLogQueue, push(_)).Times(1);

    EXPECT_CALL(motor1, set(0)).Times(AtLeast(1));
    EXPECT_CALL(motor2, set(0)).Times(AtLeast(1));
    EXPECT_CALL(motor3, set(0)).Times(AtLeast(1));
    EXPECT_CALL(motor4, set(0)).Times(AtLeast(1));

    AttitudeManager am(&mockSystemUtils, &mockMathUtils, &mockGPS, &mockIMU, &mockFFT, &mockRangefinder, &mockBarometer, &mockAMQueue, &mockTMQueue, &mockLogQueue, &motorGroup);

    failRcBit();

    am.amUpdate();
}

TEST_F(AttitudeManagerQuadTest, FailsafeRecovery) {
    RCMotorControlMessage_t rcMsg;
    rcMsg.roll = 50.0f;
    rcMsg.pitch = 50.0f;
    rcMsg.yaw = 50.0f;
    rcMsg.throttle = 50.0f;
    rcMsg.arm = true;
    rcMsg.flightMode = FlightMode_e::ACRO;

    EXPECT_CALL(mockAMQueue, count(_))
        .WillOnce(DoAll(SetArgReferee<0>(0), Return(ZP_ERROR_OK)))
        .WillRepeatedly(DoAll(SetArgReferee<0>(1), Return(ZP_ERROR_OK)));
    EXPECT_CALL(mockAMQueue, get(_))
        .WillRepeatedly(DoAll(SetArgPointee<0>(rcMsg), Return(ZP_ERROR_OK)));

    EXPECT_CALL(mockLogQueue, push(_)).Times(2);

    AttitudeManager am(&mockSystemUtils, &mockMathUtils, &mockGPS, &mockIMU, &mockFFT, &mockRangefinder, &mockBarometer, &mockAMQueue, &mockTMQueue, &mockLogQueue, &motorGroup);

    failRcBit();
    am.amUpdate();   // logs "Failsafe triggered"

    clearRcBit();
    am.amUpdate();   // logs "Motor control restored"
}

TEST_F(AttitudeManagerQuadTest, MotorClampingUpper) {
    RCMotorControlMessage_t rcMsg;
    rcMsg.roll = 50.0f;
    rcMsg.pitch = 50.0f;
    rcMsg.yaw = 50.0f;
    rcMsg.throttle = 150.0f;
    rcMsg.arm = true;
    rcMsg.flightMode = FlightMode_e::ACRO;

    EXPECT_CALL(mockAMQueue, count(_)).WillOnce(DoAll(SetArgReferee<0>(1), Return(ZP_ERROR_OK)));
    EXPECT_CALL(mockAMQueue, get(_)).WillOnce(DoAll(SetArgPointee<0>(rcMsg), Return(ZP_ERROR_OK)));

    // We use Ge(90) here because the motor output is clamped to 95% nominally due to ESC headroom
    EXPECT_CALL(motor1, set(Ge(90)));
    EXPECT_CALL(motor2, set(Ge(90)));
    EXPECT_CALL(motor3, set(Ge(90)));
    EXPECT_CALL(motor4, set(Ge(90)));

    AttitudeManager am(&mockSystemUtils, &mockMathUtils, &mockGPS, &mockIMU, &mockFFT, &mockRangefinder, &mockBarometer, &mockAMQueue, &mockTMQueue, &mockLogQueue, &motorGroup);

    am.amUpdate();
}
