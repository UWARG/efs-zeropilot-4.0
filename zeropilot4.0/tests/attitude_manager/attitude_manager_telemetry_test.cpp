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

using ::testing::_;
using ::testing::Return;
using ::testing::Invoke;
using ::testing::NiceMock;

class AttitudeManagerTelemetryTest : public ::testing::Test {
protected:
    NiceMock<MockSystemUtils> mockSystemUtils;
    NiceMock<MockFFT> mockFFT;
    NiceMock<MockGPS> mockGPS;
    NiceMock<MockIMU> mockIMU;
    NiceMock<MockMessageQueue<RCMotorControlMessage_t>> mockAMQueue;
    NiceMock<MockMessageQueue<TMMessage_t>> mockTMQueue;
    NiceMock<MockMessageQueue<char[100]>> mockLogQueue;

    NiceMock<MockMotorControl> mockMotor1;
    NiceMock<MockMotorControl> mockMotor2;
    NiceMock<MockMotorControl> mockMotor3;
    NiceMock<MockMotorControl> mockMotor4;

    // generic 4 motor setup since telemetry tests dont involve motors
    MotorInstance_t motorInstances[4] = {
        {&mockMotor1},
        {&mockMotor2},
        {&mockMotor3},
        {&mockMotor4}
    };

    MotorGroupInstance_t motorGroup{motorInstances, 4};

    void SetUp() override {
        ZP_PARAM::init();

        ON_CALL(mockSystemUtils, getCurrentTimestampMs()).WillByDefault(Return(1000));
        ON_CALL(mockIMU, readRawData()).WillByDefault(Return(RawImuBatch_t{}));      // Empty batch, count 0
        ON_CALL(mockIMU, scaleIMUData(_)).WillByDefault(Return(ScaledImuBatch_t{})); // Empty batch, count 0
        ON_CALL(mockGPS, readData()).WillByDefault(Return(GpsData_t{}));
        ON_CALL(mockAMQueue, count()).WillByDefault(Return(0));
        ON_CALL(mockTMQueue, push(_)).WillByDefault(Return(0));
        ON_CALL(mockFFT, init(_)).WillByDefault(Return(true));
    }
};

TEST_F(AttitudeManagerTelemetryTest, RawIMUTelemetrySent) {
    RawImu_t rawImu;
    rawImu.xacc = 100;
    rawImu.yacc = -200;
    rawImu.zacc = 1000;
    rawImu.xgyro = 50;
    rawImu.ygyro = -50;
    rawImu.zgyro = 25;

    RawImuBatch_t rawImuBatch{&rawImu, 1};
    EXPECT_CALL(mockIMU, readRawData()).WillRepeatedly(Return(rawImuBatch));

    int rawImuCount = 0;
    EXPECT_CALL(mockTMQueue, push(_))
        .WillRepeatedly(Invoke([&rawImuCount](TMMessage_t* msg) {
            if (msg->dataType == TMMessage_t::RAW_IMU_DATA) {
                rawImuCount++;
            }
            return 0;
        }));

    AttitudeManager am(&mockSystemUtils, &mockGPS, &mockIMU, &mockFFT, &mockAMQueue, &mockTMQueue, &mockLogQueue, &motorGroup);

    for (int i = 0; i < AM_SCHEDULING_RATE_HZ; i++) {
        am.amUpdate();
    }

    EXPECT_EQ(rawImuCount, AM_TELEMETRY_RAW_IMU_DATA_RATE_HZ);
}

TEST_F(AttitudeManagerTelemetryTest, AttitudeTelemetrySent) {
    ScaledImu_t scaledImu;
    scaledImu.xacc = 0.1f;
    scaledImu.yacc = -0.2f;
    scaledImu.zacc = 1.0f;
    scaledImu.xgyro = 0.05f;
    scaledImu.ygyro = -0.05f;
    scaledImu.zgyro = 0.025f;

    ScaledImuBatch_t scaledImuBatch{&scaledImu, 1};
    EXPECT_CALL(mockIMU, scaleIMUData(_)).WillRepeatedly(Return(scaledImuBatch));

    int attitudeCount = 0;
    EXPECT_CALL(mockTMQueue, push(_))
        .WillRepeatedly(Invoke([&attitudeCount](TMMessage_t* msg) {
            if (msg->dataType == TMMessage_t::ATTITUDE_DATA) {
                attitudeCount++;
            }
            return 0;
        }));

    AttitudeManager am(&mockSystemUtils, &mockGPS, &mockIMU, &mockFFT, &mockAMQueue, &mockTMQueue, &mockLogQueue, &motorGroup);

    for (int i = 0; i < AM_SCHEDULING_RATE_HZ; i++) {
        am.amUpdate();
    }

    EXPECT_EQ(attitudeCount, AM_TELEMETRY_ATTITUDE_DATA_RATE_HZ);
}

TEST_F(AttitudeManagerTelemetryTest, RawGPSTelemetrySent) {
    GpsData_t gpsData;
    gpsData.time = {23, 3, 15, 12, 30, 45};
    gpsData.latitude = 43.6532f;
    gpsData.longitude = -79.3832f;
    gpsData.groundSpeed = 500;
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

    AttitudeManager am(&mockSystemUtils, &mockGPS, &mockIMU, &mockFFT, &mockAMQueue, &mockTMQueue, &mockLogQueue, &motorGroup);

    for (int i = 0; i < AM_SCHEDULING_RATE_HZ; i++) {
        am.amUpdate();
    }

    EXPECT_EQ(gpsCount, AM_TELEMETRY_GPS_DATA_RATE_HZ);
}

TEST_F(AttitudeManagerTelemetryTest, ServoOutputRawTelemetrySent) {
    int servoOutputCount = 0;
    EXPECT_CALL(mockTMQueue, push(_))
        .WillRepeatedly(Invoke([&servoOutputCount](TMMessage_t* msg) {
            if (msg->dataType == TMMessage_t::SERVO_OUTPUT_RAW) {
                servoOutputCount++;
            }
            return 0;
        }));

    AttitudeManager am(&mockSystemUtils, &mockGPS, &mockIMU, &mockFFT, &mockAMQueue, &mockTMQueue, &mockLogQueue, &motorGroup);

    for (int i = 0; i < AM_SCHEDULING_RATE_HZ; i++) {
        am.amUpdate();
    }

    EXPECT_EQ(servoOutputCount, AM_TELEMETRY_SERVO_OUTPUT_RAW_RATE_HZ);
}
