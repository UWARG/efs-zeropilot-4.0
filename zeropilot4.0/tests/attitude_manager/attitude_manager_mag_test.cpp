#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "attitude_manager.hpp"
#include "mag_cal.hpp"
#include "zp_params.hpp"
#include "zp_bit.hpp"
#include "mock_systemutils.hpp"
#include "mock_gps.hpp"
#include "mock_imu.hpp"
#include "mock_magnetometer.hpp"
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

static constexpr uint32_t MAG_FAIL_MS = BIT_CONFIG[static_cast<uint16_t>(ZP_BIT_ID::MAG_DATA_VALID)].failMs;

static MagData_t newSample(float x, float y, float z) {
    MagData_t data = {};
    data.x = x;
    data.y = y;
    data.z = z;
    data.isNew = true;
    return data;
}

class AttitudeManagerMagTest : public ::testing::Test {
protected:
    uint32_t nowMs = 1000;

    NiceMock<MockSystemUtils> mockSystemUtils;
    NiceMock<MockFFT> mockFFT;
    NiceMock<MockMathUtils> mockMathUtils;
    NiceMock<MockGPS> mockGPS;
    NiceMock<MockIMU> mockIMU;
    NiceMock<MockMagnetometer> mockMag;
    NiceMock<MockRangefinder> mockRangefinder;
    NiceMock<MockBarometer> mockBarometer;
    NiceMock<MockMessageQueue<RCMotorControlMessage_t>> mockAMQueue;
    NiceMock<MockMessageQueue<TMMessage_t>> mockTMQueue;
    NiceMock<MockMessageQueue<char[100]>> mockLogQueue;

    NiceMock<MockMotorControl> mockMotor1;
    NiceMock<MockMotorControl> mockMotor2;
    NiceMock<MockMotorControl> mockMotor3;
    NiceMock<MockMotorControl> mockMotor4;

    // generic 4 motor setup since mag tests dont involve motors
    MotorInstance_t motorInstances[4] = {
        {&mockMotor1},
        {&mockMotor2},
        {&mockMotor3},
        {&mockMotor4}
    };

    MotorGroupInstance_t motorGroup{motorInstances, 4};

    void SetUp() override {
        ON_CALL(mockSystemUtils, getCurrentTimestampMs()).WillByDefault(Invoke([this]() { return nowMs; }));

        (void)ZP_BIT::init(&mockSystemUtils);
        (void)ZP_PARAM::init();

        ON_CALL(mockIMU, readRawData(_)).WillByDefault(DoAll(SetArgReferee<0>(RawImuBatch_t{}), Return(ZP_ERROR_OK)));      // Empty batch, count 0
        ON_CALL(mockIMU, scaleIMUData(_, _)).WillByDefault(DoAll(SetArgReferee<1>(ScaledImuBatch_t{}), Return(ZP_ERROR_OK))); // Empty batch, count 0
        ON_CALL(mockGPS, readData(_)).WillByDefault(DoAll(SetArgReferee<0>(GpsData_t{}), Return(ZP_ERROR_OK)));
        ON_CALL(mockMag, readData(_)).WillByDefault(DoAll(SetArgReferee<0>(MagData_t{}), Return(ZP_ERROR_OK))); // isNew false
        ON_CALL(mockBarometer, readData(_)).WillByDefault(Return(ZP_ERROR_OK));
        ON_CALL(mockAMQueue, count(_)).WillByDefault(DoAll(SetArgReferee<0>(0), Return(ZP_ERROR_OK)));
        ON_CALL(mockTMQueue, push(_)).WillByDefault(Return(ZP_ERROR_OK));
        ON_CALL(mockFFT, init(_)).WillByDefault(Return(ZP_ERROR_OK));
        ON_CALL(mockRangefinder, init()).WillByDefault(Return(ZP_ERROR_OK));
    }

    BitState_e magBitState() {
        BitState_e state = BitState_e::UNKNOWN;
        (void)ZP_BIT::getLive(ZP_BIT_ID::MAG_DATA_VALID, state);
        return state;
    }
};

TEST_F(AttitudeManagerMagTest, MagBitPassesOnNewSample) {
    ON_CALL(mockMag, readData(_)).WillByDefault(DoAll(SetArgReferee<0>(newSample(20.0f, 0.0f, 40.0f)), Return(ZP_ERROR_OK)));

    AttitudeManager am(&mockSystemUtils, &mockMathUtils, &mockGPS, &mockIMU, &mockMag, &mockFFT, &mockRangefinder, &mockBarometer, &mockAMQueue, &mockTMQueue, &mockLogQueue, &motorGroup);
    am.amUpdate();

    EXPECT_EQ(magBitState(), BitState_e::SUCCESS);
}

TEST_F(AttitudeManagerMagTest, MagBitFailsWhenSamplesStopArriving) {
    AttitudeManager am(&mockSystemUtils, &mockMathUtils, &mockGPS, &mockIMU, &mockMag, &mockFFT, &mockRangefinder, &mockBarometer, &mockAMQueue, &mockTMQueue, &mockLogQueue, &motorGroup);

    am.amUpdate();
    EXPECT_NE(magBitState(), BitState_e::FAILURE); // a single stale cycle is within the persistence window

    nowMs += MAG_FAIL_MS + 1;
    am.amUpdate();

    EXPECT_EQ(magBitState(), BitState_e::FAILURE);

    ZP_Error lastError;
    (void)ZP_BIT::getError(ZP_BIT_ID::MAG_DATA_VALID, lastError);
    EXPECT_EQ(lastError, ZP_ERROR_INVALID_DATA);
}

TEST_F(AttitudeManagerMagTest, MagBitFailsOnDriverError) {
    ON_CALL(mockMag, readData(_)).WillByDefault(DoAll(SetArgReferee<0>(MagData_t{}), Return(ZP_ERROR_EXT_API | ZP_ERROR_NACK)));

    AttitudeManager am(&mockSystemUtils, &mockMathUtils, &mockGPS, &mockIMU, &mockMag, &mockFFT, &mockRangefinder, &mockBarometer, &mockAMQueue, &mockTMQueue, &mockLogQueue, &motorGroup);
    am.amUpdate();
    nowMs += MAG_FAIL_MS + 1;
    am.amUpdate();

    EXPECT_EQ(magBitState(), BitState_e::FAILURE);

    ZP_Error lastError;
    (void)ZP_BIT::getError(ZP_BIT_ID::MAG_DATA_VALID, lastError);
    EXPECT_NE((lastError & ZP_ERROR_NACK), ZP_ERROR_OK);
}

TEST_F(AttitudeManagerMagTest, MagBitRecoversWhenSamplesResume) {
    AttitudeManager am(&mockSystemUtils, &mockMathUtils, &mockGPS, &mockIMU, &mockMag, &mockFFT, &mockRangefinder, &mockBarometer, &mockAMQueue, &mockTMQueue, &mockLogQueue, &motorGroup);
    am.amUpdate();
    nowMs += MAG_FAIL_MS + 1;
    am.amUpdate();
    ASSERT_EQ(magBitState(), BitState_e::FAILURE);

    ON_CALL(mockMag, readData(_)).WillByDefault(DoAll(SetArgReferee<0>(newSample(20.0f, 0.0f, 40.0f)), Return(ZP_ERROR_OK)));
    am.amUpdate();
    nowMs += BIT_CONFIG[static_cast<uint16_t>(ZP_BIT_ID::MAG_DATA_VALID)].clearMs + 1;
    am.amUpdate();

    EXPECT_EQ(magBitState(), BitState_e::SUCCESS);
}

TEST(MagCalTest, UpdatePassesDriverErrorThrough) {
    NiceMock<MockMagnetometer> mockMag;
    ON_CALL(mockMag, readData(_)).WillByDefault(DoAll(SetArgReferee<0>(MagData_t{}), Return(ZP_ERROR_EXT_API | ZP_ERROR_TIMEOUT)));

    MagCal magCal(&mockMag);
    MagData_t data = {};

    EXPECT_EQ(magCal.update(data), ZP_ERROR_EXT_API | ZP_ERROR_TIMEOUT);
    EXPECT_FALSE(data.isNew);
}

TEST(MagCalTest, UpdateAppliesCorrection) {
    NiceMock<MockMagnetometer> mockMag;
    ON_CALL(mockMag, readData(_)).WillByDefault(DoAll(SetArgReferee<0>(newSample(15.0f, -5.0f, 40.0f)), Return(ZP_ERROR_OK)));

    MagCal magCal(&mockMag);
    MagCalConstants_t constants = {
        {5.0f, -5.0f, 0.0f},
        {2.0f, 0.0f, 0.0f,
         0.0f, 1.0f, 0.0f,
         0.0f, 0.0f, 0.5f},
        50.0f,
    };
    ASSERT_EQ(magCal.setConstants(constants), ZP_ERROR_OK);

    MagData_t data = {};
    ASSERT_EQ(magCal.update(data), ZP_ERROR_OK);

    EXPECT_TRUE(data.isNew);
    EXPECT_FLOAT_EQ(data.x, 20.0f);
    EXPECT_FLOAT_EQ(data.y, 0.0f);
    EXPECT_FLOAT_EQ(data.z, 20.0f);
}

TEST(MagCalTest, NullDriverReportsNullptr) {
    MagCal magCal(nullptr);
    MagData_t data = {};

    EXPECT_EQ(magCal.update(data), ZP_ERROR_NULLPTR);
    EXPECT_FALSE(data.isNew);
}

TEST(MagCalTest, StartWhileCollectingIsBusy) {
    NiceMock<MockMagnetometer> mockMag;
    MagCal magCal(&mockMag);

    EXPECT_EQ(magCal.startCalibration(), ZP_ERROR_OK);
    EXPECT_EQ(magCal.startCalibration(), ZP_ERROR_BUSY);

    MagCalStatus_t status = {};
    ASSERT_EQ(magCal.getStatus(status), ZP_ERROR_OK);
    EXPECT_EQ(status.state, MAG_CAL_STATE_COLLECTING);

    EXPECT_EQ(magCal.cancelCalibration(), ZP_ERROR_OK);
    ASSERT_EQ(magCal.getStatus(status), ZP_ERROR_OK);
    EXPECT_EQ(status.state, MAG_CAL_STATE_IDLE);
}

TEST(MagCalTest, TooFewSamplesFailsCalibration) {
    NiceMock<MockMagnetometer> mockMag;
    MagData_t sample = newSample(20.0f, 0.0f, 40.0f);
    ON_CALL(mockMag, readData(_)).WillByDefault(Invoke([&sample](MagData_t &out) {
        out = sample;
        return ZP_ERROR_OK;
    }));

    MagCal magCal(&mockMag);
    ASSERT_EQ(magCal.startCalibration(), ZP_ERROR_OK);

    // Two samples spanning the whole window: the fit runs and rejects them
    MagData_t data = {};
    sample.timestamp = 0;
    ASSERT_EQ(magCal.update(data), ZP_ERROR_OK);
    sample.timestamp = MagCal::COLLECT_DURATION_MS;
    ASSERT_EQ(magCal.update(data), ZP_ERROR_OK);

    MagCalStatus_t status = {};
    ASSERT_EQ(magCal.getStatus(status), ZP_ERROR_OK);
    EXPECT_EQ(status.state, MAG_CAL_STATE_FAILED);
    EXPECT_EQ(status.error, MAG_CAL_ERR_TOO_FEW_SAMPLES);

    // A failed fit leaves the identity correction in place
    MagCalConstants_t constants = {};
    ASSERT_EQ(magCal.getConstants(constants), ZP_ERROR_OK);
    EXPECT_FLOAT_EQ(constants.softIron[0], 1.0f);
    EXPECT_FLOAT_EQ(constants.hardIron[0], 0.0f);
}
