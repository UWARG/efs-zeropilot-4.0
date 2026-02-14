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
