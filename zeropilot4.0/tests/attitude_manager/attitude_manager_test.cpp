#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "attitude_manager.hpp"
#include "zp_params.hpp"
#include "mock_systemutils.hpp"
#include "mock_gps.hpp"
#include "mock_imu.hpp"
#include "mock_queue.hpp"
#include "mock_motor.hpp"
#include "mock_barometer.hpp"

using ::testing::_;
using ::testing::Return;
using ::testing::DoAll;
using ::testing::SetArgPointee;
using ::testing::AtLeast;
using ::testing::Invoke;
using ::testing::NiceMock;

class AttitudeManagerTest : public ::testing::Test {
protected:
    int AM_RC_FAILSAFE_ITERATIONS;
    
    NiceMock<MockSystemUtils> mockSystemUtils;
    NiceMock<MockGPS> mockGPS;
    NiceMock<MockIMU> mockIMU;
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
    }; // Remaining fields overwritten by loadServoParams() from ZP_PARAM
    
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
        ON_CALL(mockIMU, readRawData()).WillByDefault(Return(RawImu_t{0, 0, 0, 0, 0, 0}));
        ON_CALL(mockIMU, scaleIMUData(_)).WillByDefault(Return(ScaledImu_t{0, 0, 0, 0, 0, 0}));
        ON_CALL(mockGPS, readData()).WillByDefault(Return(GpsData_t{}));
        ON_CALL(mockBarometer, readData(_))
        .WillByDefault(DoAll(
            SetArgPointee<0>(BaroData_t{0.0f, 0.0f, 0.0f}),
            Return(true)
        ));
        ON_CALL(mockAMQueue, count()).WillByDefault(Return(0));
        ON_CALL(mockTMQueue, push(_)).WillByDefault(Return(0));
    }
};

TEST_F(AttitudeManagerTest, MotorOutputTest) {
    RCMotorControlMessage_t rcMsg;
    rcMsg.roll = 60.0f;
    rcMsg.pitch = 70.0f;
    rcMsg.yaw = 55.0f;
    rcMsg.throttle = 80.0f;
    rcMsg.arm = true;
    rcMsg.flapAngle = 30.0f;
    rcMsg.flightMode = PlaneFlightMode_e::MANUAL;

    EXPECT_CALL(mockAMQueue, count()).WillOnce(Return(1));
    EXPECT_CALL(mockAMQueue, get(_)).WillOnce(DoAll(SetArgPointee<0>(rcMsg), Return(0)));
    
    EXPECT_CALL(mockRollMotor, set(_)).Times(AtLeast(1));
    EXPECT_CALL(mockPitchMotor, set(_)).Times(AtLeast(1));
    EXPECT_CALL(mockYawMotor, set(_)).Times(AtLeast(1));
    EXPECT_CALL(mockThrottleMotor, set(_)).Times(AtLeast(1));
    EXPECT_CALL(mockFlapMotor, set(_)).Times(AtLeast(1));
    EXPECT_CALL(mockSteeringMotor, set(_)).Times(AtLeast(1));
    
    AttitudeManager am(&mockSystemUtils, &mockGPS, &mockIMU, &mockBarometer, &mockAMQueue, &mockTMQueue, &mockLogQueue, &motorGroup);
    
    am.amUpdate();
}

TEST_F(AttitudeManagerTest, DisarmThrottleZero) {
    RCMotorControlMessage_t rcMsg;
    rcMsg.roll = 50.0f;
    rcMsg.pitch = 50.0f;
    rcMsg.yaw = 50.0f;
    rcMsg.throttle = 80.0f;
    rcMsg.arm = false;
    rcMsg.flapAngle = 0.0f;
    rcMsg.flightMode = PlaneFlightMode_e::MANUAL;

    EXPECT_CALL(mockAMQueue, count()).WillOnce(Return(1));
    EXPECT_CALL(mockAMQueue, get(_)).WillOnce(DoAll(SetArgPointee<0>(rcMsg), Return(0)));
    
    EXPECT_CALL(mockThrottleMotor, set(0));
    
    AttitudeManager am(&mockSystemUtils, &mockGPS, &mockIMU, &mockBarometer, &mockAMQueue, &mockTMQueue, &mockLogQueue, &motorGroup);
    
    am.amUpdate();
}

TEST_F(AttitudeManagerTest, FailsafeTriggered) {
    EXPECT_CALL(mockAMQueue, count()).WillRepeatedly(Return(0));
    EXPECT_CALL(mockLogQueue, push(_)).Times(1);
    
    EXPECT_CALL(mockRollMotor, set(50)).Times(AtLeast(1));
    EXPECT_CALL(mockPitchMotor, set(50)).Times(AtLeast(1));
    EXPECT_CALL(mockYawMotor, set(50)).Times(AtLeast(1));
    EXPECT_CALL(mockThrottleMotor, set(0)).Times(AtLeast(1));
    EXPECT_CALL(mockFlapMotor, set(0)).Times(AtLeast(1));
    EXPECT_CALL(mockSteeringMotor, set(50)).Times(AtLeast(1));
    
    AttitudeManager am(&mockSystemUtils, &mockGPS, &mockIMU, &mockBarometer, &mockAMQueue, &mockTMQueue, &mockLogQueue, &motorGroup);
    
    for (int i = 0; i < AM_RC_FAILSAFE_ITERATIONS; i++) {
        am.amUpdate();
    }
}

TEST_F(AttitudeManagerTest, FailsafeRecovery) {
    RCMotorControlMessage_t rcMsg;
    rcMsg.roll = 50.0f;
    rcMsg.pitch = 50.0f;
    rcMsg.yaw = 50.0f;
    rcMsg.throttle = 50.0f;
    rcMsg.arm = true;
    rcMsg.flapAngle = 0.0f;
    rcMsg.flightMode = PlaneFlightMode_e::MANUAL;
    
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
    
    AttitudeManager am(&mockSystemUtils, &mockGPS, &mockIMU, &mockBarometer, &mockAMQueue, &mockTMQueue, &mockLogQueue, &motorGroup);
    
    for (int i = 0; i < AM_RC_FAILSAFE_ITERATIONS; i++) {
        am.amUpdate();
    }
    
    am.amUpdate();
}

TEST_F(AttitudeManagerTest, MotorTrimApplied) {
    ZP_PARAM::setParamById("SERVO1_TRIM", 1550);  // 1550 us -> 55%
    
    RCMotorControlMessage_t rcMsg;
    rcMsg.roll = 50.0f;
    rcMsg.pitch = 50.0f;
    rcMsg.yaw = 50.0f;
    rcMsg.throttle = 50.0f;
    rcMsg.arm = true;
    rcMsg.flapAngle = 0.0f;
    rcMsg.flightMode = PlaneFlightMode_e::MANUAL;
    
    EXPECT_CALL(mockAMQueue, count()).WillOnce(Return(1));
    EXPECT_CALL(mockAMQueue, get(_)).WillOnce(DoAll(SetArgPointee<0>(rcMsg), Return(0)));
    
    uint8_t rollValue = 0;
    EXPECT_CALL(mockRollMotor, set(_)).WillOnce(Invoke([&rollValue](uint8_t val) { rollValue = val; }));
    
    AttitudeManager am(&mockSystemUtils, &mockGPS, &mockIMU, &mockBarometer, &mockAMQueue, &mockTMQueue, &mockLogQueue, &motorGroup);
    
    am.amUpdate();
    
    EXPECT_GT(rollValue, 50);
}

TEST_F(AttitudeManagerTest, MotorInverted) {
    ZP_PARAM::setParamById("SERVO1_REVERSED", 1);
    
    RCMotorControlMessage_t rcMsg;
    rcMsg.roll = 30.0f;
    rcMsg.pitch = 50.0f;
    rcMsg.yaw = 50.0f;
    rcMsg.throttle = 50.0f;
    rcMsg.arm = true;
    rcMsg.flapAngle = 0.0f;
    rcMsg.flightMode = PlaneFlightMode_e::MANUAL;
    
    EXPECT_CALL(mockAMQueue, count()).WillOnce(Return(1));
    EXPECT_CALL(mockAMQueue, get(_)).WillOnce(DoAll(SetArgPointee<0>(rcMsg), Return(0)));
    
    uint8_t rollValue = 0;
    EXPECT_CALL(mockRollMotor, set(_)).WillOnce(Invoke([&rollValue](uint8_t val) { rollValue = val; }));
    
    AttitudeManager am(&mockSystemUtils, &mockGPS, &mockIMU, &mockBarometer, &mockAMQueue, &mockTMQueue, &mockLogQueue, &motorGroup);
    
    am.amUpdate();
    
    EXPECT_GT(rollValue, 50);
}

TEST_F(AttitudeManagerTest, MotorClampingUpper) {
    RCMotorControlMessage_t rcMsg;
    rcMsg.roll = 150.0f;
    rcMsg.pitch = 50.0f;
    rcMsg.yaw = 50.0f;
    rcMsg.throttle = 50.0f;
    rcMsg.arm = true;
    rcMsg.flapAngle = 0.0f;
    rcMsg.flightMode = PlaneFlightMode_e::MANUAL;
    
    EXPECT_CALL(mockAMQueue, count()).WillOnce(Return(1));
    EXPECT_CALL(mockAMQueue, get(_)).WillOnce(DoAll(SetArgPointee<0>(rcMsg), Return(0)));
    
    EXPECT_CALL(mockRollMotor, set(100));
    
    AttitudeManager am(&mockSystemUtils, &mockGPS, &mockIMU, &mockBarometer, &mockAMQueue, &mockTMQueue, &mockLogQueue, &motorGroup);
    
    am.amUpdate();
}

TEST_F(AttitudeManagerTest, RawIMUTelemetrySent) {
    RawImu_t rawImu;
    rawImu.xacc = 100;
    rawImu.yacc = -200;
    rawImu.zacc = 1000;
    rawImu.xgyro = 50;
    rawImu.ygyro = -50;
    rawImu.zgyro = 25;

    EXPECT_CALL(mockIMU, readRawData()).WillRepeatedly(Return(rawImu));
    
    int rawImuCount = 0;
    EXPECT_CALL(mockTMQueue, push(_))
        .WillRepeatedly(Invoke([&rawImuCount](TMMessage_t* msg) {
            if (msg->dataType == TMMessage_t::RAW_IMU_DATA) {
                rawImuCount++;
            }
            return 0;
        }));
    
    AttitudeManager am(&mockSystemUtils, &mockGPS, &mockIMU, &mockBarometer, &mockAMQueue, &mockTMQueue, &mockLogQueue, &motorGroup);
    
    for (int i = 0; i < AM_SCHEDULING_RATE_HZ; i++) {
        am.amUpdate();
    }
    
    EXPECT_EQ(rawImuCount, AM_TELEMETRY_RAW_IMU_DATA_RATE_HZ);
}

TEST_F(AttitudeManagerTest, AttitudeTelemetrySent) {
    ScaledImu_t scaledImu;
    scaledImu.xacc = 0.1f;
    scaledImu.yacc = -0.2f;
    scaledImu.zacc = 1.0f;
    scaledImu.xgyro = 0.05f;
    scaledImu.ygyro = -0.05f;
    scaledImu.zgyro = 0.025f;

    EXPECT_CALL(mockIMU, scaleIMUData(_)).WillRepeatedly(Return(scaledImu));
    
    int attitudeCount = 0;
    EXPECT_CALL(mockTMQueue, push(_))
        .WillRepeatedly(Invoke([&attitudeCount](TMMessage_t* msg) {
            if (msg->dataType == TMMessage_t::ATTITUDE_DATA) {
                attitudeCount++;
            }
            return 0;
        }));
    
    AttitudeManager am(&mockSystemUtils, &mockGPS, &mockIMU, &mockBarometer, &mockAMQueue, &mockTMQueue, &mockLogQueue, &motorGroup);
    
    for (int i = 0; i < AM_SCHEDULING_RATE_HZ; i++) {
        am.amUpdate();
    }
    
    EXPECT_EQ(attitudeCount, AM_TELEMETRY_ATTITUDE_DATA_RATE_HZ);
}

TEST_F(AttitudeManagerTest, RawGPSTelemetrySent) {
    GpsData_t gpsData;
    gpsData.time = {23, 3, 15, 12, 30, 45};
    gpsData.latitude = 43.6532f;
    gpsData.longitude = -79.3832f;
    gpsData.groundSpeed = 500; // 5 m/s
    gpsData.numSatellites = 8;
    gpsData.altitude = 100.0f;
    gpsData.trackAngle = 90.0f;
    gpsData.isNew = true;
    gpsData.vx = 0.0f;
    gpsData.vy = 5.0f;
    gpsData.vz = 0.0f;
    
    EXPECT_CALL(mockGPS, readData()).WillRepeatedly(Return(gpsData));
    
    int gpsCount = 0;
    EXPECT_CALL(mockTMQueue, push(_))
        .WillRepeatedly(Invoke([&gpsCount](TMMessage_t* msg) {
            if (msg->dataType == TMMessage_t::GPS_RAW_DATA) {
                gpsCount++;
            }
            return 0;
        }));
    
    AttitudeManager am(&mockSystemUtils, &mockGPS, &mockIMU, &mockBarometer, &mockAMQueue, &mockTMQueue, &mockLogQueue, &motorGroup);
    
    for (int i = 0; i < AM_SCHEDULING_RATE_HZ; i++) {
        am.amUpdate();
    }
    
    EXPECT_EQ(gpsCount, AM_TELEMETRY_GPS_DATA_RATE_HZ);
}

TEST_F(AttitudeManagerTest, ServoOutputRawTelemetrySent) {
    int servoOutputCount = 0;
    EXPECT_CALL(mockTMQueue, push(_))
        .WillRepeatedly(Invoke([&servoOutputCount](TMMessage_t* msg) {
            if (msg->dataType == TMMessage_t::SERVO_OUTPUT_RAW) {
                servoOutputCount++;
            }
            return 0;
        }));
    
    AttitudeManager am(&mockSystemUtils, &mockGPS, &mockIMU, &mockBarometer, &mockAMQueue, &mockTMQueue, &mockLogQueue, &motorGroup);
    
    for (int i = 0; i < AM_SCHEDULING_RATE_HZ; i++) {
        am.amUpdate();
    }
    
    EXPECT_EQ(servoOutputCount, AM_TELEMETRY_SERVO_OUTPUT_RAW_RATE_HZ);
}

