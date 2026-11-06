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
using ::testing::DoAll;
using ::testing::SetArgReferee;
using ::testing::Invoke;
using ::testing::NiceMock;

class AttitudeManagerTelemetryTest : public ::testing::Test {
protected:
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
        (void)ZP_BIT::init(&mockSystemUtils);
        ZP_PARAM::init();

        ON_CALL(mockSystemUtils, getCurrentTimestampMs()).WillByDefault(Return(1000));
        ON_CALL(mockIMU, readRawData(_)).WillByDefault(DoAll(SetArgReferee<0>(RawImuBatch_t{}), Return(ZP_ERROR_OK)));      // Empty batch, count 0
        ON_CALL(mockIMU, scaleIMUData(_, _)).WillByDefault(DoAll(SetArgReferee<1>(ScaledImuBatch_t{}), Return(ZP_ERROR_OK))); // Empty batch, count 0
        ON_CALL(mockGPS, readData(_)).WillByDefault(DoAll(SetArgReferee<0>(GpsData_t{}), Return(ZP_ERROR_OK)));
        ON_CALL(mockBarometer, readData(_)).WillByDefault(Return(ZP_ERROR_OK));
        ON_CALL(mockAMQueue, count(_)).WillByDefault(DoAll(SetArgReferee<0>(0), Return(ZP_ERROR_OK)));
        ON_CALL(mockTMQueue, push(_)).WillByDefault(Return(ZP_ERROR_OK));
        ON_CALL(mockFFT, init(_)).WillByDefault(Return(ZP_ERROR_OK));
        ON_CALL(mockRangefinder, init()).WillByDefault(Return(ZP_ERROR_OK));
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
    EXPECT_CALL(mockIMU, readRawData(_)).WillRepeatedly(DoAll(SetArgReferee<0>(rawImuBatch), Return(ZP_ERROR_OK)));

    int rawImuCount = 0;
    EXPECT_CALL(mockTMQueue, push(_))
        .WillRepeatedly(Invoke([&rawImuCount](TMMessage_t* msg) {
            if (msg->dataType == TMMessage_t::RAW_IMU_DATA) {
                rawImuCount++;
            }
            return ZP_ERROR_OK;
        }));

    AttitudeManager am(&mockSystemUtils, &mockMathUtils, &mockGPS, &mockIMU, &mockFFT, &mockRangefinder, &mockBarometer, &mockAMQueue, &mockTMQueue, &mockLogQueue, &motorGroup);

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
    EXPECT_CALL(mockIMU, scaleIMUData(_, _)).WillRepeatedly(DoAll(SetArgReferee<1>(scaledImuBatch), Return(ZP_ERROR_OK)));

    int attitudeCount = 0;
    EXPECT_CALL(mockTMQueue, push(_))
        .WillRepeatedly(Invoke([&attitudeCount](TMMessage_t* msg) {
            if (msg->dataType == TMMessage_t::ATTITUDE_DATA) {
                attitudeCount++;
            }
            return ZP_ERROR_OK;
        }));

    AttitudeManager am(&mockSystemUtils, &mockMathUtils, &mockGPS, &mockIMU, &mockFFT, &mockRangefinder, &mockBarometer, &mockAMQueue, &mockTMQueue, &mockLogQueue, &motorGroup);

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

    EXPECT_CALL(mockGPS, readData(_)).WillRepeatedly(DoAll(SetArgReferee<0>(gpsData), Return(ZP_ERROR_OK)));

    int gpsCount = 0;
    EXPECT_CALL(mockTMQueue, push(_))
        .WillRepeatedly(Invoke([&gpsCount](TMMessage_t* msg) {
            if (msg->dataType == TMMessage_t::GPS_RAW_DATA) {
                gpsCount++;
            }
            return ZP_ERROR_OK;
        }));

    AttitudeManager am(&mockSystemUtils, &mockMathUtils, &mockGPS, &mockIMU, &mockFFT, &mockRangefinder, &mockBarometer, &mockAMQueue, &mockTMQueue, &mockLogQueue, &motorGroup);

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
            return ZP_ERROR_OK;
        }));

    AttitudeManager am(&mockSystemUtils, &mockMathUtils, &mockGPS, &mockIMU, &mockFFT, &mockRangefinder, &mockBarometer, &mockAMQueue, &mockTMQueue, &mockLogQueue, &motorGroup);

    for (int i = 0; i < AM_SCHEDULING_RATE_HZ; i++) {
        am.amUpdate();
    }

    EXPECT_EQ(servoOutputCount, AM_TELEMETRY_SERVO_OUTPUT_RAW_RATE_HZ);
}

TEST_F(AttitudeManagerTelemetryTest, ScaledPressureTelemetrySent) {
    BaroData_t baroData;
    baroData.pressureKPa = 101.325f;
    baroData.temperatureC = 25.0f;

    EXPECT_CALL(mockBarometer, readData(_))
        .WillRepeatedly(Invoke([&baroData](BaroData_t& outData) {
            outData = baroData;
            return ZP_ERROR_OK;
        }));

    int pressureCount = 0;
    EXPECT_CALL(mockTMQueue, push(_))
        .WillRepeatedly(Invoke([&pressureCount](TMMessage_t* msg) {
            if (msg->dataType == TMMessage_t::SCALED_PRESSURE_DATA) {
                pressureCount++;
            }
            return ZP_ERROR_OK;
        }));

    AttitudeManager am(&mockSystemUtils, &mockMathUtils, &mockGPS, &mockIMU, &mockFFT, &mockRangefinder, &mockBarometer, &mockAMQueue, &mockTMQueue, &mockLogQueue, &motorGroup);

    for (int i = 0; i < AM_SCHEDULING_RATE_HZ; i++) {
        am.amUpdate();
    }

    EXPECT_EQ(pressureCount, AM_TELEMETRY_SCALED_PRESSURE_DATA_RATE_HZ);
}
