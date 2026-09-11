#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <cstring>
#include "system_manager.hpp"
#include "zp_params.hpp"
#include "zp_bit.hpp"
#include "mock_systemutils.hpp"
#include "mock_iwdg.hpp"
#include "mock_logger.hpp"
#include "mock_rc.hpp"
#include "mock_power_module.hpp"
#include "mock_queue.hpp"

using ::testing::_;
using ::testing::Return;
using ::testing::Invoke;
using ::testing::NiceMock;
using ::testing::DoAll;
using ::testing::SetArgReferee;

// ZP_PARAM::get reports through an out param; these tests only need the value
static float paramValue(ZP_PARAM_ID id) {
    float value = 0.0f;
    (void)ZP_PARAM::get(id, value);
    return value;
}

class SystemManagerTest : public ::testing::Test {
protected:
    int RC_FAILSAFE_ITERATIONS;
    
    NiceMock<MockSystemUtils> mockSystemUtils;
    NiceMock<MockWatchdog> mockWatchdog;
    NiceMock<MockLogger> mockLogger;
    ISafetySwitch* mockSafetySwitchPtr = nullptr; // Safety switch is not used in unit tests by default
    NiceMock<MockRCReceiver> mockRC;
    NiceMock<MockPowerModule> mockPM;
    NiceMock<MockMessageQueue<RCMotorControlMessage_t>> mockAMQueue;
    NiceMock<MockMessageQueue<TMMessage_t>> mockTMQueue;
    NiceMock<MockMessageQueue<char[100]>> mockLogQueue;

    void SetUp() override {
        (void)ZP_PARAM::init();
        // BIT holds a static clock pointer and static state, so re-point and reset it per test
        (void)ZP_BIT::init(&mockSystemUtils);

        RC_FAILSAFE_ITERATIONS =
            ((paramValue(ZP_PARAM_ID::RC_FS_TIMEOUT) * 1000) / SM_UPDATE_LOOP_DELAY_MS) + 5;
    }
};

TEST_F(SystemManagerTest, WatchdogRefresh) {
    EXPECT_CALL(mockWatchdog, refreshWatchdog()).Times(1);
    
    SystemManager sm(&mockSystemUtils, &mockWatchdog, &mockLogger, mockSafetySwitchPtr,
                     &mockRC, &mockPM, &mockAMQueue, &mockTMQueue, &mockLogQueue);
    
    sm.smUpdate();
}

TEST_F(SystemManagerTest, RCFailsafeStopsForwarding) {
    RCControl validRCData;
    validRCData.isDataNew = true;
    validRCData.roll = 50.0f;
    validRCData.pitch = 50.0f;
    validRCData.yaw = 50.0f;
    validRCData.throttle = 50.0f;
    validRCData.arm = 100.0f;

    RCControl staleRCData = validRCData;
    staleRCData.isDataNew = false;

    EXPECT_CALL(mockRC, getRCData(_))
        .WillOnce(DoAll(SetArgReferee<0>(validRCData), Return(ZP_ERROR_OK)))
        .WillRepeatedly(DoAll(SetArgReferee<0>(staleRCData), Return(ZP_ERROR_OK)));

    EXPECT_CALL(mockAMQueue, push(_)).Times(1); 

    SystemManager sm(&mockSystemUtils, &mockWatchdog, &mockLogger, mockSafetySwitchPtr,
                     &mockRC, &mockPM, &mockAMQueue, &mockTMQueue, &mockLogQueue);

    sm.smUpdate();

    for (int i = 0; i < RC_FAILSAFE_ITERATIONS; i++) {
        sm.smUpdate();
    }
}

TEST_F(SystemManagerTest, HeartbeatSentToTelemetry) {  
    int heartbeatCount = 0;
    EXPECT_CALL(mockTMQueue, push(_))
        .WillRepeatedly(::testing::Invoke([&heartbeatCount](TMMessage_t* msg) {
            if (msg->dataType == TMMessage_t::HEARTBEAT_DATA) {
                heartbeatCount++;
            }
            return ZP_ERROR_OK;
        }));
    
    SystemManager sm(&mockSystemUtils, &mockWatchdog, &mockLogger, mockSafetySwitchPtr,
                     &mockRC, &mockPM, &mockAMQueue, &mockTMQueue, &mockLogQueue);
    
    for (int i = 0; i < SM_SCHEDULING_RATE_HZ; i++) {
        sm.smUpdate();
    }
    
    EXPECT_EQ(heartbeatCount, SM_TELEMETRY_HEARTBEAT_RATE_HZ);
}

TEST_F(SystemManagerTest, RCDataSentToTelemetry) {
    RCControl rcData;
    rcData.isDataNew = true;
    rcData.roll = 60.0f;
    rcData.pitch = 70.0f;
    
    EXPECT_CALL(mockRC, getRCData(_)).WillRepeatedly(DoAll(SetArgReferee<0>(rcData), Return(ZP_ERROR_OK)));
    
    int rcDataCount = 0;
    EXPECT_CALL(mockTMQueue, push(_))
        .WillRepeatedly(::testing::Invoke([&rcDataCount](TMMessage_t* msg) {
            if (msg->dataType == TMMessage_t::RC_DATA) {
                rcDataCount++;
            }
            return ZP_ERROR_OK;
        }));
    
    SystemManager sm(&mockSystemUtils, &mockWatchdog, &mockLogger, mockSafetySwitchPtr,
                     &mockRC, &mockPM, &mockAMQueue, &mockTMQueue, &mockLogQueue);
    
    for (int i = 0; i < SM_SCHEDULING_RATE_HZ; i++) {
        sm.smUpdate();
    }
    
    EXPECT_EQ(rcDataCount, SM_TELEMETRY_RC_DATA_RATE_HZ);
}

TEST_F(SystemManagerTest, BatteryDataSentToTelemetry) {
    EXPECT_CALL(mockPM, readData(_)).WillRepeatedly(Return(ZP_ERROR_OK));

    int batteryDataCount = 0;
    EXPECT_CALL(mockTMQueue, push(_))
        .WillRepeatedly(::testing::Invoke([&batteryDataCount](TMMessage_t* msg) {
            if (msg->dataType == TMMessage_t::BATTERY_DATA) {
                batteryDataCount++;
            }
            return ZP_ERROR_OK;
        }));
    
    SystemManager sm(&mockSystemUtils, &mockWatchdog, &mockLogger, mockSafetySwitchPtr,
                     &mockRC, &mockPM, &mockAMQueue, &mockTMQueue, &mockLogQueue);
    
    for (int i = 0; i < SM_SCHEDULING_RATE_HZ; i++) {
        sm.smUpdate();
    }
    
    EXPECT_EQ(batteryDataCount, SM_TELEMETRY_BATTERY_DATA_RATE_HZ);    
}

TEST_F(SystemManagerTest, BatteryLowDetection) {

    // Voltage safely inside LOW band:
    // BATT_LOW_VOLT <= V < BATT_CRT_VOLT
    EXPECT_CALL(mockPM, readData(_))
        .WillRepeatedly(::testing::Invoke([](PMData_t* data) {
            data->busVoltage =
                (paramValue(ZP_PARAM_ID::BATT_LOW_VOLT) + paramValue(ZP_PARAM_ID::BATT_CRT_VOLT)) / 2.0f;
            data->current = 1.0f;
            data->charge = 0;
            data->energy = 0;
            return ZP_ERROR_OK;
        }));

    bool sawLow = false;

    EXPECT_CALL(mockTMQueue, push(_))
        .WillRepeatedly(::testing::Invoke([&](TMMessage_t* msg) {
            if (msg->dataType == TMMessage_t::BATTERY_DATA &&
                msg->tmMessageData.batteryData.chargeState ==
                MAV_BATTERY_CHARGE_STATE_LOW) {
                sawLow = true;
            }
            return ZP_ERROR_OK;
        }));

    SystemManager sm(&mockSystemUtils, &mockWatchdog, &mockLogger, mockSafetySwitchPtr,
                     &mockRC, &mockPM, &mockAMQueue, &mockTMQueue, &mockLogQueue);

    const int loopsToLow =
        (paramValue(ZP_PARAM_ID::BATT_LOW_TIMER) * 1000) / SM_UPDATE_LOOP_DELAY_MS; // number of loops to transition to low state

    const int totalLoops =
        loopsToLow + SM_SCHEDULING_RATE_HZ;  // one extra cycle to ensure telemetry boundary

    for (int i = 0; i < totalLoops; i++) {
        sm.smUpdate();
    }

    EXPECT_TRUE(sawLow);
}


TEST_F(SystemManagerTest, BatteryCritDetection) {

    // Voltage safely below CRITICAL threshold
    // V < BATT_CRT_VOLT
    EXPECT_CALL(mockPM, readData(_))
        .WillRepeatedly(::testing::Invoke([](PMData_t* data) {
            data->busVoltage =
                paramValue(ZP_PARAM_ID::BATT_CRT_VOLT) - 0.1f;
            data->current = 1.0f;
            data->charge = 0;
            data->energy = 0;
            return ZP_ERROR_OK;
        }));

    bool sawCritical = false;

    EXPECT_CALL(mockTMQueue, push(_))
        .WillRepeatedly(::testing::Invoke([&](TMMessage_t* msg) {
            if (msg->dataType == TMMessage_t::BATTERY_DATA &&
                msg->tmMessageData.batteryData.chargeState ==
                MAV_BATTERY_CHARGE_STATE_CRITICAL) {
                sawCritical = true;
            }
            return ZP_ERROR_OK;
        }));

    SystemManager sm(&mockSystemUtils, &mockWatchdog, &mockLogger, mockSafetySwitchPtr,
                     &mockRC, &mockPM, &mockAMQueue, &mockTMQueue, &mockLogQueue);

    const int loopsToCritical =
        (paramValue(ZP_PARAM_ID::BATT_LOW_TIMER) * 1000) / SM_UPDATE_LOOP_DELAY_MS; // number of loops to transition to critical state

    const int totalLoops =
        loopsToCritical + SM_SCHEDULING_RATE_HZ;  // one extra cycle to ensure telemetry boundary

    for (int i = 0; i < totalLoops; i++) {
        sm.smUpdate();
    }

    EXPECT_TRUE(sawCritical);
}

TEST_F(SystemManagerTest, RCFlightmodeSwitching) {
    // Helper to scale μs to the float values used by decodeRawFlightMode
    auto scalePWM = [](float pwm) { return (pwm - 1000.0f) / 10.0f; };

    // Internal test mapping: Nominal PWM -> Expected Enum from SystemManager constants
    struct {
        float pwm;
        FlightMode_e expected;
    } testCases[] = {
        {1165.0f, static_cast<FlightMode_e>(static_cast<uint32_t>(paramValue(ZP_PARAM_ID::FLTMODE1)))},
        {1295.0f, static_cast<FlightMode_e>(static_cast<uint32_t>(paramValue(ZP_PARAM_ID::FLTMODE2)))},
        {1425.0f, static_cast<FlightMode_e>(static_cast<uint32_t>(paramValue(ZP_PARAM_ID::FLTMODE3)))},
        {1555.0f, static_cast<FlightMode_e>(static_cast<uint32_t>(paramValue(ZP_PARAM_ID::FLTMODE4)))},
        {1685.0f, static_cast<FlightMode_e>(static_cast<uint32_t>(paramValue(ZP_PARAM_ID::FLTMODE5)))},
        {1815.0f, static_cast<FlightMode_e>(static_cast<uint32_t>(paramValue(ZP_PARAM_ID::FLTMODE6)))}
    };

    SystemManager sm(&mockSystemUtils, &mockWatchdog, &mockLogger, mockSafetySwitchPtr, &mockRC, 
                     &mockPM, &mockAMQueue, &mockTMQueue, &mockLogQueue);

    for (const auto& test : testCases) {
        RCControl rcData;
        rcData.isDataNew = true;
        rcData.fltModeRaw = scalePWM(test.pwm);
        rcData.arm = 100.0f; // Armed to ensure data flows

        // Expect the RC driver to return our test value
        EXPECT_CALL(mockRC, getRCData(_)).WillOnce(DoAll(SetArgReferee<0>(rcData), Return(ZP_ERROR_OK)));

        // Verify the exact enum reaches the Attitude Manager queue
        EXPECT_CALL(mockAMQueue, push(::testing::Field(&RCMotorControlMessage_t::flightMode, test.expected)))
            .Times(1);

        sm.smUpdate();

        // Reset mocks for the next button case
        ::testing::Mock::VerifyAndClearExpectations(&mockRC);
        ::testing::Mock::VerifyAndClearExpectations(&mockAMQueue);
    }
}

TEST_F(SystemManagerTest, CriticalReportBitSendsTextWithoutDisarmFailsafe) {
    // BIT debounce needs a clock that moves
    uint32_t nowMs = 0;
    ON_CALL(mockSystemUtils, getCurrentTimestampMs()).WillByDefault(Invoke([&nowMs]() { return nowMs; }));

    // Keep the battery healthy so IMU_DATA_VALID is the only failing BIT
    ON_CALL(mockPM, readData(_)).WillByDefault(Invoke([](PMData_t* data) {
        data->busVoltage = paramValue(ZP_PARAM_ID::BATT_LOW_VOLT) + 5.0f;
        return ZP_ERROR_OK;
    }));

    RCControl rc;
    rc.arm = 100.0f;
    rc.isDataNew = true;
    ON_CALL(mockRC, getRCData(_)).WillByDefault(DoAll(SetArgReferee<0>(rc), Return(ZP_ERROR_OK)));

    int failTexts = 0;
    int disarmTexts = 0;
    uint8_t failSeverity = 0;
    ON_CALL(mockTMQueue, push(_)).WillByDefault(Invoke([&](TMMessage_t* msg) {
        if (msg->dataType == TMMessage_t::STATUSTEXT_DATA) {
            if (std::strcmp(msg->tmMessageData.statusTextData.text, "PreArm: IMU data invalid") == 0) {
                failTexts++;
                failSeverity = msg->tmMessageData.statusTextData.severity;
            } else if (std::strcmp(msg->tmMessageData.statusTextData.text, "Disarming") == 0) {
                disarmTexts++;
            }
        }
        return ZP_ERROR_OK;
    }));

    bool lastArm = true;
    ON_CALL(mockAMQueue, push(_)).WillByDefault(Invoke([&lastArm](RCMotorControlMessage_t* msg) {
        lastArm = msg->arm;
        return ZP_ERROR_OK;
    }));

    SystemManager sm(&mockSystemUtils, &mockWatchdog, &mockLogger, mockSafetySwitchPtr,
                     &mockRC, &mockPM, &mockAMQueue, &mockTMQueue, &mockLogQueue);

    // IMU_DATA_VALID is CRITICAL with a REPORT failsafe. AM owns it, so fail it directly past its debounce
    (void)ZP_BIT::report(ZP_BIT_ID::IMU_DATA_VALID, ZP_ERROR_FAIL);
    nowMs += BIT_CONFIG[static_cast<uint16_t>(ZP_BIT_ID::IMU_DATA_VALID)].failMs;
    (void)ZP_BIT::report(ZP_BIT_ID::IMU_DATA_VALID, ZP_ERROR_FAIL);

    const int REPORT_PERIOD_TICKS = SM_TELEMETRY_BIT_FAIL_PERIOD_S * SM_SCHEDULING_RATE_HZ;
    const int TICKS = 2 * REPORT_PERIOD_TICKS + SM_SCHEDULING_RATE_HZ;
    for (int i = 0; i < TICKS; i++) {
        sm.smUpdate();
        nowMs += SM_UPDATE_LOOP_DELAY_MS;
    }

    EXPECT_GE(failTexts, 2) << "the failure text repeats while the BIT is failing";
    EXPECT_LE(failTexts, TICKS / REPORT_PERIOD_TICKS + 1) << "and is rate limited";
    EXPECT_EQ(failSeverity, MAV_SEVERITY_CRITICAL) << "a CRITICAL BIT reports at critical severity";
    EXPECT_EQ(disarmTexts, 0) << "a REPORT failsafe must not trigger the disarm action";
    EXPECT_FALSE(lastArm) << "a CRITICAL BIT still blocks arming through the pre-arm check";
}
