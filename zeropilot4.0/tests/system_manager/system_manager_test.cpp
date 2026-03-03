#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "system_manager.hpp"
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

class SystemManagerTest : public ::testing::Test {
protected:
    static constexpr int RC_FAILSAFE_ITERATIONS = (SM_RC_TIMEOUT_MS / SM_UPDATE_LOOP_DELAY_MS) + 5;
    
    NiceMock<MockSystemUtils> mockSystemUtils;
    NiceMock<MockWatchdog> mockWatchdog;
    NiceMock<MockLogger> mockLogger;
    NiceMock<MockRCReceiver> mockRC;
    NiceMock<MockPowerModule> mockPM;
    NiceMock<MockMessageQueue<RCMotorControlMessage_t>> mockAMQueue;
    NiceMock<MockMessageQueue<TMMessage_t>> mockTMQueue;
    NiceMock<MockMessageQueue<char[100]>> mockLogQueue;
};

TEST_F(SystemManagerTest, WatchdogRefresh) {
    EXPECT_CALL(mockWatchdog, refreshWatchdog()).Times(1);
    
    SystemManager sm(&mockSystemUtils, &mockWatchdog, &mockLogger, &mockRC, &mockPM,
                     &mockAMQueue, &mockTMQueue, &mockLogQueue);
    
    sm.smUpdate();
}

TEST_F(SystemManagerTest, RCFailsafeStopsForwarding) {
    RCControl validRCData;
    validRCData.isDataNew = true;
    validRCData.roll = 50.0f;
    validRCData.pitch = 50.0f;
    validRCData.yaw = 50.0f;
    validRCData.throttle = 50.0f;
    validRCData.arm = 1.0f;

    RCControl staleRCData = validRCData;
    staleRCData.isDataNew = false;

    EXPECT_CALL(mockRC, getRCData())
        .WillOnce(Return(validRCData))
        .WillRepeatedly(Return(staleRCData));

    EXPECT_CALL(mockAMQueue, push(_)).Times(1); 

    SystemManager sm(&mockSystemUtils, &mockWatchdog, &mockLogger, &mockRC, &mockPM,
                     &mockAMQueue, &mockTMQueue, &mockLogQueue);

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
            return 0;
        }));
    
    SystemManager sm(&mockSystemUtils, &mockWatchdog, &mockLogger, &mockRC, &mockPM,
                     &mockAMQueue, &mockTMQueue, &mockLogQueue);
    
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
    
    EXPECT_CALL(mockRC, getRCData()).WillRepeatedly(Return(rcData));
    
    int rcDataCount = 0;
    EXPECT_CALL(mockTMQueue, push(_))
        .WillRepeatedly(::testing::Invoke([&rcDataCount](TMMessage_t* msg) {
            if (msg->dataType == TMMessage_t::RC_DATA) {
                rcDataCount++;
            }
            return 0;
        }));
    
    SystemManager sm(&mockSystemUtils, &mockWatchdog, &mockLogger, &mockRC, &mockPM,
                     &mockAMQueue, &mockTMQueue, &mockLogQueue);
    
    for (int i = 0; i < SM_SCHEDULING_RATE_HZ; i++) {
        sm.smUpdate();
    }
    
    EXPECT_EQ(rcDataCount, SM_TELEMETRY_RC_DATA_RATE_HZ);
}

TEST_F(SystemManagerTest, BatteryDataSentToTelemetry) {
    EXPECT_CALL(mockPM, readData(_)).WillRepeatedly(Return(true));

    int batteryDataCount = 0;
    EXPECT_CALL(mockTMQueue, push(_))
        .WillRepeatedly(::testing::Invoke([&batteryDataCount](TMMessage_t* msg) {
            if (msg->dataType == TMMessage_t::BATTERY_DATA) {
                batteryDataCount++;
            }
            return 0;
        }));
    
    SystemManager sm(&mockSystemUtils, &mockWatchdog, &mockLogger, &mockRC, &mockPM,
                     &mockAMQueue, &mockTMQueue, &mockLogQueue);
    
    for (int i = 0; i < SM_SCHEDULING_RATE_HZ; i++) {
        sm.smUpdate();
    }
    
    EXPECT_EQ(batteryDataCount, SM_TELEMETRY_BATTERY_DATA_RATE_HZ);    
}

TEST_F(SystemManagerTest, BatteryLowDetection) {

    // Voltage safely inside LOW band:
    // BATTERY_CRITICAL_VOLTAGE <= V < BATTERY_LOW_VOLTAGE
    EXPECT_CALL(mockPM, readData(_))
        .WillRepeatedly(::testing::Invoke([](PMData_t* data) {
            data->busVoltage =
                (BATTERY_LOW_VOLTAGE + BATTERY_CRITICAL_VOLTAGE) / 2.0f;
            data->current = 1.0f;
            data->charge = 0;
            data->energy = 0;
            return true;
        }));

    bool sawLow = false;

    EXPECT_CALL(mockTMQueue, push(_))
        .WillRepeatedly(::testing::Invoke([&](TMMessage_t* msg) {
            if (msg->dataType == TMMessage_t::BATTERY_DATA &&
                msg->tmMessageData.batteryData.chargeState ==
                MAV_BATTERY_CHARGE_STATE_LOW) {
                sawLow = true;
            }
            return 0;
        }));

    SystemManager sm(&mockSystemUtils, &mockWatchdog, &mockLogger,
                     &mockRC, &mockPM,
                     &mockAMQueue, &mockTMQueue, &mockLogQueue);

    const int loopsToLow =
        SM_BATTERY_LOW_TIME_MS / SM_UPDATE_LOOP_DELAY_MS; // number of loops to transition to low state

    const int totalLoops =
        loopsToLow + SM_SCHEDULING_RATE_HZ;  // one extra cycle to ensure telemetry boundary

    for (int i = 0; i < totalLoops; i++) {
        sm.smUpdate();
    }

    EXPECT_TRUE(sawLow);
}


TEST_F(SystemManagerTest, BatteryCritDetection) {

    // Voltage safely below CRITICAL threshold
    // V < BATTERY_CRITICAL_VOLTAGE
    EXPECT_CALL(mockPM, readData(_))
        .WillRepeatedly(::testing::Invoke([](PMData_t* data) {
            data->busVoltage =
                BATTERY_CRITICAL_VOLTAGE - 0.1f;
            data->current = 1.0f;
            data->charge = 0;
            data->energy = 0;
            return true;
        }));

    bool sawCritical = false;

    EXPECT_CALL(mockTMQueue, push(_))
        .WillRepeatedly(::testing::Invoke([&](TMMessage_t* msg) {
            if (msg->dataType == TMMessage_t::BATTERY_DATA &&
                msg->tmMessageData.batteryData.chargeState ==
                MAV_BATTERY_CHARGE_STATE_CRITICAL) {
                sawCritical = true;
            }
            return 0;
        }));

    SystemManager sm(&mockSystemUtils, &mockWatchdog, &mockLogger,
                     &mockRC, &mockPM,
                     &mockAMQueue, &mockTMQueue, &mockLogQueue);

    const int loopsToCritical =
        SM_BATTERY_CRITICAL_TIME_MS / SM_UPDATE_LOOP_DELAY_MS; // number of loops to transition to critical state

    const int totalLoops =
        loopsToCritical + SM_SCHEDULING_RATE_HZ;  // one extra cycle to ensure telemetry boundary

    for (int i = 0; i < totalLoops; i++) {
        sm.smUpdate();
    }

    EXPECT_TRUE(sawCritical);
}
