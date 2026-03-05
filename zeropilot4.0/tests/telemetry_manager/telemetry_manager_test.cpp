#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "telemetry_manager.hpp"
#include "mock_systemutils.hpp"
#include "mock_telemlink.hpp"
#include "mock_queue.hpp"

using ::testing::_;
using ::testing::Return;
using ::testing::DoAll;
using ::testing::SetArgPointee;

class TelemetryManagerTest : public ::testing::Test {
protected:
    MockSystemUtils mockSystemUtils;
    MockTelemLink mockTelemLink;
    MockMessageQueue<TMMessage_t> mockTMQueue;
    MockMessageQueue<RCMotorControlMessage_t> mockAMQueue;
    MockMessageQueue<mavlink_message_t> mockPackedMsgBuffer;
};

TEST_F(TelemetryManagerTest, HeartbeatProcessing) {
    TMMessage_t hbMsg = heartbeatPack(1000, MAV_MODE_FLAG_SAFETY_ARMED, 0, MAV_STATE_ACTIVE);
    
    EXPECT_CALL(mockTMQueue, count()).WillOnce(Return(1)).WillRepeatedly(Return(0));
    EXPECT_CALL(mockTMQueue, get(_)).WillOnce(DoAll(SetArgPointee<0>(hbMsg), Return(0)));
    EXPECT_CALL(mockPackedMsgBuffer, push(_)).Times(1);
    EXPECT_CALL(mockPackedMsgBuffer, count()).WillRepeatedly(Return(0));
    EXPECT_CALL(mockTelemLink, receive(_, _)).WillOnce(Return(0));
    
    TelemetryManager tm(&mockSystemUtils, &mockTelemLink, &mockTMQueue, &mockAMQueue, &mockPackedMsgBuffer);
    tm.tmUpdate();
}

TEST_F(TelemetryManagerTest, StatusTextProcessing) {
    TMMessage_t stMsg = {};
    stMsg.dataType = TMMessage_t::STATUSTEXT_DATA;
    stMsg.tmMessageData.statusTextData.severity = MAV_SEVERITY_CRITICAL;
    snprintf(stMsg.tmMessageData.statusTextData.text, 50, "CRITICAL ERROR");
    stMsg.tmMessageData.statusTextData.id = 1;
    stMsg.tmMessageData.statusTextData.chunkSeq = 0;

    EXPECT_CALL(mockTMQueue, count()).WillOnce(Return(1)).WillRepeatedly(Return(0));
    EXPECT_CALL(mockTMQueue, get(_)).WillOnce(DoAll(SetArgPointee<0>(stMsg), Return(0)));
    EXPECT_CALL(mockPackedMsgBuffer, push(_)).Times(1);
    EXPECT_CALL(mockPackedMsgBuffer, count()).WillRepeatedly(Return(0));
    EXPECT_CALL(mockTelemLink, receive(_, _)).WillOnce(Return(0));

    TelemetryManager tm(&mockSystemUtils, &mockTelemLink, &mockTMQueue, &mockAMQueue, &mockPackedMsgBuffer);
    tm.tmUpdate();
}

TEST_F(TelemetryManagerTest, GPSRawDataProcessing) {
    TMMessage_t gpsMsg = gpsRawDataPack(1000, 3, 436532000, -793832000, 100000, 100, 100, 500, 9000, 8);
    
    EXPECT_CALL(mockTMQueue, count()).WillOnce(Return(1)).WillRepeatedly(Return(0));
    EXPECT_CALL(mockTMQueue, get(_)).WillOnce(DoAll(SetArgPointee<0>(gpsMsg), Return(0)));
    EXPECT_CALL(mockPackedMsgBuffer, push(_)).Times(1);
    EXPECT_CALL(mockPackedMsgBuffer, count()).WillRepeatedly(Return(0));
    EXPECT_CALL(mockTelemLink, receive(_, _)).WillOnce(Return(0));
    
    TelemetryManager tm(&mockSystemUtils, &mockTelemLink, &mockTMQueue, &mockAMQueue, &mockPackedMsgBuffer);
    tm.tmUpdate();
}

TEST_F(TelemetryManagerTest, ServoOutputRawProcessing) {
    TMMessage_t servoMsg = {};
    servoMsg.dataType = TMMessage_t::SERVO_OUTPUT_RAW;
    servoMsg.timeBootMs = 1234;
    servoMsg.tmMessageData.servoOutputRawData.port = 0;
    servoMsg.tmMessageData.servoOutputRawData.servo1Raw = 1500;
    servoMsg.tmMessageData.servoOutputRawData.servo2Raw = 1100;

    EXPECT_CALL(mockTMQueue, count()).WillOnce(Return(1)).WillRepeatedly(Return(0));
    EXPECT_CALL(mockTMQueue, get(_)).WillOnce(DoAll(SetArgPointee<0>(servoMsg), Return(0)));
    EXPECT_CALL(mockPackedMsgBuffer, push(_)).Times(1);
    EXPECT_CALL(mockPackedMsgBuffer, count()).WillRepeatedly(Return(0));
    EXPECT_CALL(mockTelemLink, receive(_, _)).WillOnce(Return(0));

    TelemetryManager tm(&mockSystemUtils, &mockTelemLink, &mockTMQueue, &mockAMQueue, &mockPackedMsgBuffer);
    tm.tmUpdate();
}

TEST_F(TelemetryManagerTest, RCDataProcessing) {
    TMMessage_t rcMsg = rcDataPack(1000, 50, 50, 50, 50, 0, 1);
    
    EXPECT_CALL(mockTMQueue, count()).WillOnce(Return(1)).WillRepeatedly(Return(0));
    EXPECT_CALL(mockTMQueue, get(_)).WillOnce(DoAll(SetArgPointee<0>(rcMsg), Return(0)));
    EXPECT_CALL(mockPackedMsgBuffer, push(_)).Times(1);
    EXPECT_CALL(mockPackedMsgBuffer, count()).WillRepeatedly(Return(0));
    EXPECT_CALL(mockTelemLink, receive(_, _)).WillOnce(Return(0));
    
    TelemetryManager tm(&mockSystemUtils, &mockTelemLink, &mockTMQueue, &mockAMQueue, &mockPackedMsgBuffer);
    tm.tmUpdate();
}

TEST_F(TelemetryManagerTest, BatteryDataProcessing_Normal) {
    TMMessage_t batMsg = {};
    batMsg.dataType = TMMessage_t::BATTERY_DATA;
    batMsg.tmMessageData.batteryData.batteryId = 0;
    batMsg.tmMessageData.batteryData.chargeState = MAV_BATTERY_CHARGE_STATE_OK; 
    batMsg.tmMessageData.batteryData.temperature = 2500; // 25°C
    batMsg.tmMessageData.batteryData.currentBattery = 500;

    EXPECT_CALL(mockTMQueue, count()).WillOnce(Return(1)).WillRepeatedly(Return(0));
    EXPECT_CALL(mockTMQueue, get(_)).WillOnce(DoAll(SetArgPointee<0>(batMsg), Return(0)));
    EXPECT_CALL(mockPackedMsgBuffer, push(_)).Times(1);
    EXPECT_CALL(mockPackedMsgBuffer, count()).WillRepeatedly(Return(0));
    EXPECT_CALL(mockTelemLink, receive(_, _)).WillOnce(Return(0));

    TelemetryManager tm(&mockSystemUtils, &mockTelemLink, &mockTMQueue, &mockAMQueue, &mockPackedMsgBuffer);
    tm.tmUpdate();
}

TEST_F(TelemetryManagerTest, BatteryDataProcessing_CriticalFault) {
    TMMessage_t batMsg = {};
    batMsg.dataType = TMMessage_t::BATTERY_DATA;
    batMsg.tmMessageData.batteryData.batteryId = 0;
    // This triggers: faultBitmask = MAV_BATTERY_FAULT_DEEP_DISCHARGE
    batMsg.tmMessageData.batteryData.chargeState = MAV_BATTERY_CHARGE_STATE_CRITICAL; 
    batMsg.tmMessageData.batteryData.currentBattery = 1000;

    EXPECT_CALL(mockTMQueue, count()).WillOnce(Return(1)).WillRepeatedly(Return(0));
    EXPECT_CALL(mockTMQueue, get(_)).WillOnce(DoAll(SetArgPointee<0>(batMsg), Return(0)));
    EXPECT_CALL(mockPackedMsgBuffer, push(_)).Times(1);
    EXPECT_CALL(mockPackedMsgBuffer, count()).WillRepeatedly(Return(0));
    EXPECT_CALL(mockTelemLink, receive(_, _)).WillOnce(Return(0));

    TelemetryManager tm(&mockSystemUtils, &mockTelemLink, &mockTMQueue, &mockAMQueue, &mockPackedMsgBuffer);
    tm.tmUpdate();
}

TEST_F(TelemetryManagerTest, RawIMUDataProcessing) {
    TMMessage_t imuMsg = rawImuDataPack(1000, 100, -200, 1000, 50, -50, 25);
    
    EXPECT_CALL(mockTMQueue, count()).WillOnce(Return(1)).WillRepeatedly(Return(0));
    EXPECT_CALL(mockTMQueue, get(_)).WillOnce(DoAll(SetArgPointee<0>(imuMsg), Return(0)));
    EXPECT_CALL(mockPackedMsgBuffer, push(_)).Times(1);
    EXPECT_CALL(mockPackedMsgBuffer, count()).WillRepeatedly(Return(0));
    EXPECT_CALL(mockTelemLink, receive(_, _)).WillOnce(Return(0));
    
    TelemetryManager tm(&mockSystemUtils, &mockTelemLink, &mockTMQueue, &mockAMQueue, &mockPackedMsgBuffer);
    tm.tmUpdate();
}

TEST_F(TelemetryManagerTest, AttitudeDataProcessing) {
    TMMessage_t attMsg = attitudeDataPack(1000, 0.1f, -0.2f, 1.5f);
    
    EXPECT_CALL(mockTMQueue, count()).WillOnce(Return(1)).WillRepeatedly(Return(0));
    EXPECT_CALL(mockTMQueue, get(_)).WillOnce(DoAll(SetArgPointee<0>(attMsg), Return(0)));
    EXPECT_CALL(mockPackedMsgBuffer, push(_)).Times(1);
    EXPECT_CALL(mockPackedMsgBuffer, count()).WillRepeatedly(Return(0));
    EXPECT_CALL(mockTelemLink, receive(_, _)).WillOnce(Return(0));
    
    TelemetryManager tm(&mockSystemUtils, &mockTelemLink, &mockTMQueue, &mockAMQueue, &mockPackedMsgBuffer);
    tm.tmUpdate();
}

TEST_F(TelemetryManagerTest, TransmitWhenBufferNotEmpty) {
    mavlink_message_t testMsg = {};
    testMsg.len = 10;
    
    EXPECT_CALL(mockTMQueue, count()).WillRepeatedly(Return(0));
    EXPECT_CALL(mockPackedMsgBuffer, count())
        .WillOnce(Return(1))
        .WillOnce(Return(1))
        .WillRepeatedly(Return(0));
    EXPECT_CALL(mockPackedMsgBuffer, get(_)).WillOnce(DoAll(SetArgPointee<0>(testMsg), Return(0)));
    EXPECT_CALL(mockTelemLink, transmit(_, _)).Times(1);
    EXPECT_CALL(mockTelemLink, receive(_, _)).WillOnce(Return(0));
    
    TelemetryManager tm(&mockSystemUtils, &mockTelemLink, &mockTMQueue, &mockAMQueue, &mockPackedMsgBuffer);
    tm.tmUpdate();
}

TEST_F(TelemetryManagerTest, NoTransmitWhenBufferEmpty) {
    EXPECT_CALL(mockTMQueue, count()).WillRepeatedly(Return(0));
    EXPECT_CALL(mockPackedMsgBuffer, count()).WillRepeatedly(Return(0));
    EXPECT_CALL(mockTelemLink, transmit(_, _)).Times(0);
    EXPECT_CALL(mockTelemLink, receive(_, _)).WillOnce(Return(0));
    
    TelemetryManager tm(&mockSystemUtils, &mockTelemLink, &mockTMQueue, &mockAMQueue, &mockPackedMsgBuffer);
    tm.tmUpdate();
}
