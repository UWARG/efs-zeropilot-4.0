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

class AttitudeManagerPlaneTest : public ::testing::Test {
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

    NiceMock<MockMotorControl> mockRollMotor;
    NiceMock<MockMotorControl> mockPitchMotor;
    NiceMock<MockMotorControl> mockYawMotor;
    NiceMock<MockMotorControl> mockThrottleMotor;
    NiceMock<MockMotorControl> mockFlapMotor;
    NiceMock<MockMotorControl> mockSteeringMotor;

    MotorInstance_t motorInstances[6] = {
        {&mockRollMotor},
        {&mockPitchMotor},
        {&mockYawMotor},
        {&mockThrottleMotor},
        {&mockFlapMotor},
        {&mockSteeringMotor}
    }; // Remaining fields overwritten by AMParamSetup::loadAllParams() from ZP_PARAM

    MotorGroupInstance_t motorGroup{motorInstances, 6};

    void SetUp() override {
        ZP_PARAM::init();

        // Override servo params so tests are independent of zp_params defaults.
        // Test uses 6 motors: AILERON, ELEVATOR, RUDDER, THROTTLE, FLAP, GROUND_STEERING
        // Params are in PWM us: trim=1500(->50%), min=1000(->0%), max=2000(->100%)
        ZP_PARAM::setParamById("SERVO1_TRIM", 1500);
        ZP_PARAM::setParamById("SERVO1_MIN", 1000);
        ZP_PARAM::setParamById("SERVO1_MAX", 2000);
        ZP_PARAM::setParamById("SERVO1_REVERSED", 0);
        ZP_PARAM::setParamById("SERVO1_FUNCTION", static_cast<float>(MotorFunction_e::AILERON));

        ZP_PARAM::setParamById("SERVO2_TRIM", 1500);
        ZP_PARAM::setParamById("SERVO2_MIN", 1000);
        ZP_PARAM::setParamById("SERVO2_MAX", 2000);
        ZP_PARAM::setParamById("SERVO2_REVERSED", 0);
        ZP_PARAM::setParamById("SERVO2_FUNCTION", static_cast<float>(MotorFunction_e::ELEVATOR));

        ZP_PARAM::setParamById("SERVO3_TRIM", 1500);
        ZP_PARAM::setParamById("SERVO3_MIN", 1000);
        ZP_PARAM::setParamById("SERVO3_MAX", 2000);
        ZP_PARAM::setParamById("SERVO3_REVERSED", 0);
        ZP_PARAM::setParamById("SERVO3_FUNCTION", static_cast<float>(MotorFunction_e::RUDDER));

        ZP_PARAM::setParamById("SERVO4_TRIM", 1500);
        ZP_PARAM::setParamById("SERVO4_MIN", 1000);
        ZP_PARAM::setParamById("SERVO4_MAX", 2000);
        ZP_PARAM::setParamById("SERVO4_REVERSED", 0);
        ZP_PARAM::setParamById("SERVO4_FUNCTION", static_cast<float>(MotorFunction_e::THROTTLE));

        ZP_PARAM::setParamById("SERVO5_TRIM", 1500);
        ZP_PARAM::setParamById("SERVO5_MIN", 1000);
        ZP_PARAM::setParamById("SERVO5_MAX", 2000);
        ZP_PARAM::setParamById("SERVO5_REVERSED", 0);
        ZP_PARAM::setParamById("SERVO5_FUNCTION", static_cast<float>(MotorFunction_e::FLAP));

        ZP_PARAM::setParamById("SERVO6_TRIM", 1500);
        ZP_PARAM::setParamById("SERVO6_MIN", 1000);
        ZP_PARAM::setParamById("SERVO6_MAX", 2000);
        ZP_PARAM::setParamById("SERVO6_REVERSED", 0);
        ZP_PARAM::setParamById("SERVO6_FUNCTION", static_cast<float>(MotorFunction_e::GROUND_STEERING));

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

TEST_F(AttitudeManagerPlaneTest, MotorOutputTest) {
    RCMotorControlMessage_t rcMsg;
    rcMsg.roll = 60.0f;
    rcMsg.pitch = 70.0f;
    rcMsg.yaw = 55.0f;
    rcMsg.throttle = 80.0f;
    rcMsg.arm = true;
    rcMsg.flapAngle = 30.0f;
    rcMsg.flightMode = FlightMode_e::MANUAL;

    EXPECT_CALL(mockAMQueue, count(_)).WillOnce(DoAll(SetArgReferee<0>(1), Return(ZP_ERROR_OK)));
    EXPECT_CALL(mockAMQueue, get(_)).WillOnce(DoAll(SetArgPointee<0>(rcMsg), Return(ZP_ERROR_OK)));

    EXPECT_CALL(mockRollMotor, set(_)).Times(AtLeast(1));
    EXPECT_CALL(mockPitchMotor, set(_)).Times(AtLeast(1));
    EXPECT_CALL(mockYawMotor, set(_)).Times(AtLeast(1));
    EXPECT_CALL(mockThrottleMotor, set(_)).Times(AtLeast(1));
    EXPECT_CALL(mockFlapMotor, set(_)).Times(AtLeast(1));
    EXPECT_CALL(mockSteeringMotor, set(_)).Times(AtLeast(1));

    AttitudeManager am(&mockSystemUtils, &mockMathUtils, &mockGPS, &mockIMU, &mockFFT, &mockRangefinder, &mockBarometer, &mockAMQueue, &mockTMQueue, &mockLogQueue, &motorGroup);

    am.amUpdate();
}

TEST_F(AttitudeManagerPlaneTest, DisarmThrottleZero) {
    RCMotorControlMessage_t rcMsg;
    rcMsg.roll = 50.0f;
    rcMsg.pitch = 50.0f;
    rcMsg.yaw = 50.0f;
    rcMsg.throttle = 80.0f;
    rcMsg.arm = false;
    rcMsg.flapAngle = 0.0f;
    rcMsg.flightMode = FlightMode_e::MANUAL;

    EXPECT_CALL(mockAMQueue, count(_)).WillOnce(DoAll(SetArgReferee<0>(1), Return(ZP_ERROR_OK)));
    EXPECT_CALL(mockAMQueue, get(_)).WillOnce(DoAll(SetArgPointee<0>(rcMsg), Return(ZP_ERROR_OK)));

    EXPECT_CALL(mockThrottleMotor, set(0));

    AttitudeManager am(&mockSystemUtils, &mockMathUtils, &mockGPS, &mockIMU, &mockFFT, &mockRangefinder, &mockBarometer, &mockAMQueue, &mockTMQueue, &mockLogQueue, &motorGroup);

    am.amUpdate();
}

TEST_F(AttitudeManagerPlaneTest, FailsafeTriggered) {
    EXPECT_CALL(mockAMQueue, count(_)).WillRepeatedly(DoAll(SetArgReferee<0>(0), Return(ZP_ERROR_OK)));
    EXPECT_CALL(mockLogQueue, push(_)).Times(1);

    EXPECT_CALL(mockRollMotor, set(50)).Times(AtLeast(1));
    EXPECT_CALL(mockPitchMotor, set(50)).Times(AtLeast(1));
    EXPECT_CALL(mockYawMotor, set(50)).Times(AtLeast(1));
    EXPECT_CALL(mockThrottleMotor, set(0)).Times(AtLeast(1));
    EXPECT_CALL(mockFlapMotor, set(0)).Times(AtLeast(1));
    EXPECT_CALL(mockSteeringMotor, set(50)).Times(AtLeast(1));

    AttitudeManager am(&mockSystemUtils, &mockMathUtils, &mockGPS, &mockIMU, &mockFFT, &mockRangefinder, &mockBarometer, &mockAMQueue, &mockTMQueue, &mockLogQueue, &motorGroup);

    failRcBit();

    am.amUpdate();
}

TEST_F(AttitudeManagerPlaneTest, FailsafeRecovery) {
    RCMotorControlMessage_t rcMsg;
    rcMsg.roll = 50.0f;
    rcMsg.pitch = 50.0f;
    rcMsg.yaw = 50.0f;
    rcMsg.throttle = 50.0f;
    rcMsg.arm = true;
    rcMsg.flapAngle = 0.0f;
    rcMsg.flightMode = FlightMode_e::MANUAL;

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

TEST_F(AttitudeManagerPlaneTest, MotorTrimApplied) {
    ZP_PARAM::setParamById("SERVO1_TRIM", 1550);  // 1550 us -> 55%

    RCMotorControlMessage_t rcMsg;
    rcMsg.roll = 50.0f;
    rcMsg.pitch = 50.0f;
    rcMsg.yaw = 50.0f;
    rcMsg.throttle = 50.0f;
    rcMsg.arm = true;
    rcMsg.flapAngle = 0.0f;
    rcMsg.flightMode = FlightMode_e::MANUAL;

    EXPECT_CALL(mockAMQueue, count(_)).WillOnce(DoAll(SetArgReferee<0>(1), Return(ZP_ERROR_OK)));
    EXPECT_CALL(mockAMQueue, get(_)).WillOnce(DoAll(SetArgPointee<0>(rcMsg), Return(ZP_ERROR_OK)));

    uint8_t rollValue = 0;
    EXPECT_CALL(mockRollMotor, set(_)).WillOnce(Invoke([&rollValue](uint32_t val) { rollValue = val; return ZP_ERROR_OK; }));

    AttitudeManager am(&mockSystemUtils, &mockMathUtils, &mockGPS, &mockIMU, &mockFFT, &mockRangefinder, &mockBarometer, &mockAMQueue, &mockTMQueue, &mockLogQueue, &motorGroup);

    am.amUpdate();

    EXPECT_GT(rollValue, 50);
}

TEST_F(AttitudeManagerPlaneTest, MotorInverted) {
    ZP_PARAM::setParamById("SERVO1_REVERSED", 1);

    RCMotorControlMessage_t rcMsg;
    rcMsg.roll = 30.0f;
    rcMsg.pitch = 50.0f;
    rcMsg.yaw = 50.0f;
    rcMsg.throttle = 50.0f;
    rcMsg.arm = true;
    rcMsg.flapAngle = 0.0f;
    rcMsg.flightMode = FlightMode_e::MANUAL;

    EXPECT_CALL(mockAMQueue, count(_)).WillOnce(DoAll(SetArgReferee<0>(1), Return(ZP_ERROR_OK)));
    EXPECT_CALL(mockAMQueue, get(_)).WillOnce(DoAll(SetArgPointee<0>(rcMsg), Return(ZP_ERROR_OK)));

    uint8_t rollValue = 0;
    EXPECT_CALL(mockRollMotor, set(_)).WillOnce(Invoke([&rollValue](uint32_t val) { rollValue = val; return ZP_ERROR_OK; }));

    AttitudeManager am(&mockSystemUtils, &mockMathUtils, &mockGPS, &mockIMU, &mockFFT, &mockRangefinder, &mockBarometer, &mockAMQueue, &mockTMQueue, &mockLogQueue, &motorGroup);

    am.amUpdate();

    EXPECT_GT(rollValue, 50);
}

TEST_F(AttitudeManagerPlaneTest, MotorClampingUpper) {
    RCMotorControlMessage_t rcMsg;
    rcMsg.roll = 150.0f;
    rcMsg.pitch = 50.0f;
    rcMsg.yaw = 50.0f;
    rcMsg.throttle = 50.0f;
    rcMsg.arm = true;
    rcMsg.flapAngle = 0.0f;
    rcMsg.flightMode = FlightMode_e::MANUAL;

    EXPECT_CALL(mockAMQueue, count(_)).WillOnce(DoAll(SetArgReferee<0>(1), Return(ZP_ERROR_OK)));
    EXPECT_CALL(mockAMQueue, get(_)).WillOnce(DoAll(SetArgPointee<0>(rcMsg), Return(ZP_ERROR_OK)));

    EXPECT_CALL(mockRollMotor, set(100));

    AttitudeManager am(&mockSystemUtils, &mockMathUtils, &mockGPS, &mockIMU, &mockFFT, &mockRangefinder, &mockBarometer, &mockAMQueue, &mockTMQueue, &mockLogQueue, &motorGroup);

    am.amUpdate();
}
