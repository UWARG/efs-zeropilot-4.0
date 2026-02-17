#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "attitude_manager.hpp"
#include "mock_systemutils.hpp"
#include "mock_gps.hpp"
#include "mock_imu.hpp"
#include "mock_queue.hpp"
#include "mock_motor.hpp"

using ::testing::_;
using ::testing::Return;
using ::testing::DoAll;
using ::testing::SetArgPointee;
using ::testing::AtLeast;
using ::testing::Invoke;
using ::testing::NiceMock;

class AttitudeManagerTest : public ::testing::Test {
protected:
    static constexpr int AM_RC_FAILSAFE_ITERATIONS = (AM_FAILSAFE_TIMEOUT_MS / AM_UPDATE_LOOP_DELAY_MS) + 5;
    
    NiceMock<MockSystemUtils> mockSystemUtils;
    NiceMock<MockGPS> mockGPS;
    NiceMock<MockIMU> mockIMU;
    NiceMock<MockMessageQueue<RCMotorControlMessage_t>> mockAMQueue;
    NiceMock<MockMessageQueue<TMMessage_t>> mockTMQueue;
    NiceMock<MockMessageQueue<char[100]>> mockLogQueue;
    
    NiceMock<MockMotorControl> mockRollMotor;
    NiceMock<MockMotorControl> mockPitchMotor;
    NiceMock<MockMotorControl> mockYawMotor;
    NiceMock<MockMotorControl> mockThrottleMotor;
    NiceMock<MockMotorControl> mockFlapMotor;
    NiceMock<MockMotorControl> mockSteeringMotor;
    
    MotorInstance_t rollMotorInst{&mockRollMotor, false, 0};
    MotorInstance_t pitchMotorInst{&mockPitchMotor, false, 0};
    MotorInstance_t yawMotorInst{&mockYawMotor, false, 0};
    MotorInstance_t throttleMotorInst{&mockThrottleMotor, false, 0};
    MotorInstance_t flapMotorInst{&mockFlapMotor, false, 0};
    MotorInstance_t steeringMotorInst{&mockSteeringMotor, false, 0};
    
    MotorGroupInstance_t rollGroup{&rollMotorInst, 1};
    MotorGroupInstance_t pitchGroup{&pitchMotorInst, 1};
    MotorGroupInstance_t yawGroup{&yawMotorInst, 1};
    MotorGroupInstance_t throttleGroup{&throttleMotorInst, 1};
    MotorGroupInstance_t flapGroup{&flapMotorInst, 1};
    MotorGroupInstance_t steeringGroup{&steeringMotorInst, 1};
    
    void SetUp() override {
        ON_CALL(mockSystemUtils, getCurrentTimestampMs()).WillByDefault(Return(1000));
        ON_CALL(mockIMU, readRawData()).WillByDefault(Return(RawImu_t{0, 0, 0, 0, 0, 0}));
        ON_CALL(mockIMU, scaleIMUData(_)).WillByDefault(Return(ScaledImu_t{0, 0, 0, 0, 0, 0}));
        ON_CALL(mockGPS, readData()).WillByDefault(Return(GpsData_t{}));
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
    rcMsg.arm = 1.0f;
    rcMsg.flapAngle = 30.0f;

    EXPECT_CALL(mockAMQueue, count()).WillOnce(Return(1));
    EXPECT_CALL(mockAMQueue, get(_)).WillOnce(DoAll(SetArgPointee<0>(rcMsg), Return(0)));
    
    EXPECT_CALL(mockRollMotor, set(_)).Times(AtLeast(1));
    EXPECT_CALL(mockPitchMotor, set(_)).Times(AtLeast(1));
    EXPECT_CALL(mockYawMotor, set(_)).Times(AtLeast(1));
    EXPECT_CALL(mockThrottleMotor, set(_)).Times(AtLeast(1));
    EXPECT_CALL(mockFlapMotor, set(_)).Times(AtLeast(1));
    EXPECT_CALL(mockSteeringMotor, set(_)).Times(AtLeast(1));
    
    AttitudeManager am(&mockSystemUtils, &mockGPS, &mockIMU, &mockAMQueue, &mockTMQueue, &mockLogQueue,
                       &rollGroup, &pitchGroup, &yawGroup, &throttleGroup, &flapGroup, &steeringGroup);
    
    am.amUpdate();
}

TEST_F(AttitudeManagerTest, DisarmThrottleZero) {
    RCMotorControlMessage_t rcMsg;
    rcMsg.roll = 50.0f;
    rcMsg.pitch = 50.0f;
    rcMsg.yaw = 50.0f;
    rcMsg.throttle = 80.0f;
    rcMsg.arm = 0.0f;
    rcMsg.flapAngle = 0.0f;

    EXPECT_CALL(mockAMQueue, count()).WillOnce(Return(1));
    EXPECT_CALL(mockAMQueue, get(_)).WillOnce(DoAll(SetArgPointee<0>(rcMsg), Return(0)));
    
    EXPECT_CALL(mockThrottleMotor, set(0));
    
    AttitudeManager am(&mockSystemUtils, &mockGPS, &mockIMU, &mockAMQueue, &mockTMQueue, &mockLogQueue,
                       &rollGroup, &pitchGroup, &yawGroup, &throttleGroup, &flapGroup, &steeringGroup);
    
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
    
    AttitudeManager am(&mockSystemUtils, &mockGPS, &mockIMU, &mockAMQueue, &mockTMQueue, &mockLogQueue,
                       &rollGroup, &pitchGroup, &yawGroup, &throttleGroup, &flapGroup, &steeringGroup);
    
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
    rcMsg.arm = 1.0f;
    rcMsg.flapAngle = 0.0f;
    
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
    
    AttitudeManager am(&mockSystemUtils, &mockGPS, &mockIMU, &mockAMQueue, &mockTMQueue, &mockLogQueue,
                       &rollGroup, &pitchGroup, &yawGroup, &throttleGroup, &flapGroup, &steeringGroup);
    
    for (int i = 0; i < AM_RC_FAILSAFE_ITERATIONS; i++) {
        am.amUpdate();
    }
    
    am.amUpdate();
}

TEST_F(AttitudeManagerTest, MotorTrimApplied) {
    MotorInstance_t rollMotorWithTrim{&mockRollMotor, false, 5};
    MotorGroupInstance_t rollGroupWithTrim{&rollMotorWithTrim, 1};
    
    RCMotorControlMessage_t rcMsg;
    rcMsg.roll = 50.0f;
    rcMsg.pitch = 50.0f;
    rcMsg.yaw = 50.0f;
    rcMsg.throttle = 50.0f;
    rcMsg.arm = 1.0f;
    rcMsg.flapAngle = 0.0f;
    
    EXPECT_CALL(mockAMQueue, count()).WillOnce(Return(1));
    EXPECT_CALL(mockAMQueue, get(_)).WillOnce(DoAll(SetArgPointee<0>(rcMsg), Return(0)));
    
    uint8_t rollValue = 0;
    EXPECT_CALL(mockRollMotor, set(_)).WillOnce(Invoke([&rollValue](uint8_t val) { rollValue = val; }));
    
    AttitudeManager am(&mockSystemUtils, &mockGPS, &mockIMU, &mockAMQueue, &mockTMQueue, &mockLogQueue,
                       &rollGroupWithTrim, &pitchGroup, &yawGroup, &throttleGroup, &flapGroup, &steeringGroup);
    
    am.amUpdate();
    
    EXPECT_GT(rollValue, 50);
}

TEST_F(AttitudeManagerTest, MotorInverted) {
    MotorInstance_t rollMotorInverted{&mockRollMotor, true, 0};
    MotorGroupInstance_t rollGroupInverted{&rollMotorInverted, 1};
    
    RCMotorControlMessage_t rcMsg;
    rcMsg.roll = 30.0f;
    rcMsg.pitch = 50.0f;
    rcMsg.yaw = 50.0f;
    rcMsg.throttle = 50.0f;
    rcMsg.arm = 1.0f;
    rcMsg.flapAngle = 0.0f;
    
    EXPECT_CALL(mockAMQueue, count()).WillOnce(Return(1));
    EXPECT_CALL(mockAMQueue, get(_)).WillOnce(DoAll(SetArgPointee<0>(rcMsg), Return(0)));
    
    uint8_t rollValue = 0;
    EXPECT_CALL(mockRollMotor, set(_)).WillOnce(Invoke([&rollValue](uint8_t val) { rollValue = val; }));
    
    AttitudeManager am(&mockSystemUtils, &mockGPS, &mockIMU, &mockAMQueue, &mockTMQueue, &mockLogQueue,
                       &rollGroupInverted, &pitchGroup, &yawGroup, &throttleGroup, &flapGroup, &steeringGroup);
    
    am.amUpdate();
    
    EXPECT_GT(rollValue, 50);
}

TEST_F(AttitudeManagerTest, MotorClampingUpper) {
    RCMotorControlMessage_t rcMsg;
    rcMsg.roll = 150.0f;
    rcMsg.pitch = 50.0f;
    rcMsg.yaw = 50.0f;
    rcMsg.throttle = 50.0f;
    rcMsg.arm = 1.0f;
    rcMsg.flapAngle = 0.0f;
    
    EXPECT_CALL(mockAMQueue, count()).WillOnce(Return(1));
    EXPECT_CALL(mockAMQueue, get(_)).WillOnce(DoAll(SetArgPointee<0>(rcMsg), Return(0)));
    
    EXPECT_CALL(mockRollMotor, set(100));
    
    AttitudeManager am(&mockSystemUtils, &mockGPS, &mockIMU, &mockAMQueue, &mockTMQueue, &mockLogQueue,
                       &rollGroup, &pitchGroup, &yawGroup, &throttleGroup, &flapGroup, &steeringGroup);
    
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
    
    AttitudeManager am(&mockSystemUtils, &mockGPS, &mockIMU, &mockAMQueue, &mockTMQueue, &mockLogQueue,
                       &rollGroup, &pitchGroup, &yawGroup, &throttleGroup, &flapGroup, &steeringGroup);
    
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
    
    AttitudeManager am(&mockSystemUtils, &mockGPS, &mockIMU, &mockAMQueue, &mockTMQueue, &mockLogQueue,
                       &rollGroup, &pitchGroup, &yawGroup, &throttleGroup, &flapGroup, &steeringGroup);
    
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
    
    AttitudeManager am(&mockSystemUtils, &mockGPS, &mockIMU, &mockAMQueue, &mockTMQueue, &mockLogQueue,
                       &rollGroup, &pitchGroup, &yawGroup, &throttleGroup, &flapGroup, &steeringGroup);
    
    for (int i = 0; i < AM_SCHEDULING_RATE_HZ; i++) {
        am.amUpdate();
    }
    
    EXPECT_EQ(gpsCount, AM_TELEMETRY_GPS_DATA_RATE_HZ);
}
