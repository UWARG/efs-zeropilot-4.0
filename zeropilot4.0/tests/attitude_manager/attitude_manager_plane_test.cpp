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

class AttitudeManagerPlaneTest : public ::testing::Test {
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

        AM_RC_FAILSAFE_ITERATIONS =
            static_cast<int>(((ZP_PARAM::get(ZP_PARAM_ID::RC_FS_TIMEOUT)) * 1000) / AM_UPDATE_LOOP_DELAY_MS) + 5;

        ON_CALL(mockSystemUtils, getCurrentTimestampMs()).WillByDefault(Return(1000));
        ON_CALL(mockIMU, readRawData()).WillByDefault(Return(RawImuBatch_t{}));      // Empty batch, count 0
        ON_CALL(mockIMU, scaleIMUData(_)).WillByDefault(Return(ScaledImuBatch_t{})); // Empty batch, count 0
        ON_CALL(mockGPS, readData()).WillByDefault(Return(GpsData_t{}));
        ON_CALL(mockAMQueue, count()).WillByDefault(Return(0));
        ON_CALL(mockTMQueue, push(_)).WillByDefault(Return(0));
        ON_CALL(mockFFT, init(_)).WillByDefault(Return(true));
        ON_CALL(mockRangefinder, init(_)).WillByDefault(Return(true));
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

    EXPECT_CALL(mockAMQueue, count()).WillOnce(Return(1));
    EXPECT_CALL(mockAMQueue, get(_)).WillOnce(DoAll(SetArgPointee<0>(rcMsg), Return(0)));

    EXPECT_CALL(mockRollMotor, set(_)).Times(AtLeast(1));
    EXPECT_CALL(mockPitchMotor, set(_)).Times(AtLeast(1));
    EXPECT_CALL(mockYawMotor, set(_)).Times(AtLeast(1));
    EXPECT_CALL(mockThrottleMotor, set(_)).Times(AtLeast(1));
    EXPECT_CALL(mockFlapMotor, set(_)).Times(AtLeast(1));
    EXPECT_CALL(mockSteeringMotor, set(_)).Times(AtLeast(1));

    AttitudeManager am(&mockSystemUtils, &mockGPS, &mockIMU, &mockFFT, & mockRangefinder, &mockAMQueue, &mockTMQueue, &mockLogQueue, &motorGroup);

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

    EXPECT_CALL(mockAMQueue, count()).WillOnce(Return(1));
    EXPECT_CALL(mockAMQueue, get(_)).WillOnce(DoAll(SetArgPointee<0>(rcMsg), Return(0)));

    EXPECT_CALL(mockThrottleMotor, set(0));

    AttitudeManager am(&mockSystemUtils, &mockGPS, &mockIMU, &mockFFT, & mockRangefinder, &mockAMQueue, &mockTMQueue, &mockLogQueue, &motorGroup);

    am.amUpdate();
}

TEST_F(AttitudeManagerPlaneTest, FailsafeTriggered) {
    EXPECT_CALL(mockAMQueue, count()).WillRepeatedly(Return(0));
    EXPECT_CALL(mockLogQueue, push(_)).Times(1);

    EXPECT_CALL(mockRollMotor, set(50)).Times(AtLeast(1));
    EXPECT_CALL(mockPitchMotor, set(50)).Times(AtLeast(1));
    EXPECT_CALL(mockYawMotor, set(50)).Times(AtLeast(1));
    EXPECT_CALL(mockThrottleMotor, set(0)).Times(AtLeast(1));
    EXPECT_CALL(mockFlapMotor, set(0)).Times(AtLeast(1));
    EXPECT_CALL(mockSteeringMotor, set(50)).Times(AtLeast(1));

    AttitudeManager am(&mockSystemUtils, &mockGPS, &mockIMU, &mockFFT, & mockRangefinder, &mockAMQueue, &mockTMQueue, &mockLogQueue, &motorGroup);

    for (int i = 0; i < AM_RC_FAILSAFE_ITERATIONS; i++) {
        am.amUpdate();
    }
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

    EXPECT_CALL(mockAMQueue, count()).WillOnce(Return(1));
    EXPECT_CALL(mockAMQueue, get(_)).WillOnce(DoAll(SetArgPointee<0>(rcMsg), Return(0)));

    uint8_t rollValue = 0;
    EXPECT_CALL(mockRollMotor, set(_)).WillOnce(Invoke([&rollValue](uint8_t val) { rollValue = val; }));

    AttitudeManager am(&mockSystemUtils, &mockGPS, &mockIMU, &mockFFT, & mockRangefinder, &mockAMQueue, &mockTMQueue, &mockLogQueue, &motorGroup);

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

    EXPECT_CALL(mockAMQueue, count()).WillOnce(Return(1));
    EXPECT_CALL(mockAMQueue, get(_)).WillOnce(DoAll(SetArgPointee<0>(rcMsg), Return(0)));

    uint8_t rollValue = 0;
    EXPECT_CALL(mockRollMotor, set(_)).WillOnce(Invoke([&rollValue](uint8_t val) { rollValue = val; }));

    AttitudeManager am(&mockSystemUtils, &mockGPS, &mockIMU, &mockFFT, & mockRangefinder, &mockAMQueue, &mockTMQueue, &mockLogQueue, &motorGroup);

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

    EXPECT_CALL(mockAMQueue, count()).WillOnce(Return(1));
    EXPECT_CALL(mockAMQueue, get(_)).WillOnce(DoAll(SetArgPointee<0>(rcMsg), Return(0)));

    EXPECT_CALL(mockRollMotor, set(100));

    AttitudeManager am(&mockSystemUtils, &mockGPS, &mockIMU, &mockFFT, & mockRangefinder, &mockAMQueue, &mockTMQueue, &mockLogQueue, &motorGroup);

    am.amUpdate();
}
