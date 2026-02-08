#include "telemetry_manager.hpp"
#define SYSTEM_ID 1             // Suggested System ID by Mavlink
#define COMPONENT_ID 1          // Suggested Component ID by MAVLINK

#define TM_RFD_BAUDRATE 57600
#define TM_SCHEDULING_RATE_HZ 20
#define TM_RFD_TX_LOADING_FACTOR 0.8f
#define TM_MAX_TRANSMISSION_BYTES (uint16_t)(TM_RFD_TX_LOADING_FACTOR * (TM_RFD_BAUDRATE / (8 * TM_SCHEDULING_RATE_HZ)))

TelemetryManager::TelemetryManager(
    ISystemUtils *systemUtilsDriver,
    IRFD *rfdDriver,
    IMessageQueue<TMMessage_t> *tmQueueDriver,
    IMessageQueue<RCMotorControlMessage_t> *amQueueDriver,
    IMessageQueue<mavlink_message_t> *messageBuffer
) :
    systemUtilsDriver(systemUtilsDriver),
    rfdDriver(rfdDriver),
    tmQueueDriver(tmQueueDriver),
    amQueueDriver(amQueueDriver),
    messageBuffer(messageBuffer),
    overflowMsgPending(false) {}

TelemetryManager::~TelemetryManager() = default;

void TelemetryManager::tmUpdate() {
    processMsgQueue();
    transmit();
}

void TelemetryManager::processMsgQueue() {
    uint16_t count = tmQueueDriver->count();
    TMMessage rcMsg = {};
    bool rc = false;
	while (count-- > 0) {
        mavlink_message_t mavlinkMessage = {0};
        TMMessage_t tmqMessage = {};
        tmQueueDriver->get(&tmqMessage);

        switch (tmqMessage.dataType) {
            case TMMessage_t::HEARTBEAT_DATA: {
                auto heartbeatData = tmqMessage.tmMessageData.heartbeatData;
                mavlink_msg_heartbeat_pack(SYSTEM_ID, COMPONENT_ID, &mavlinkMessage, MAV_TYPE_FIXED_WING, MAV_AUTOPILOT_INVALID,
                	heartbeatData.baseMode, heartbeatData.customMode, heartbeatData.systemStatus);
                break;
            }

            case TMMessage_t::GPS_RAW_DATA: {
                auto& g = tmqMessage.tmMessageData.gpsRawData;
                mavlink_msg_gps_raw_int_pack(SYSTEM_ID, COMPONENT_ID, &mavlinkMessage, (uint64_t)tmqMessage.timeBootMs * 1000,
                    g.fixType, g.lat, g.lon, g.alt, g.eph, g.epv, g.vel, g.cog, g.satellitesVisible, g.altEllipsoid, 
                    g.hAcc, g.vAcc, g.velAcc, g.hdgAcc, g.yaw);
                break;
            }

            case TMMessage_t::RC_DATA: {
                rcMsg = tmqMessage;
                rc = true;
                continue;
            }

            case TMMessage_t::BM_DATA: {
                auto bmData = tmqMessage.tmMessageData.bmData;
                mavlink_msg_battery_status_pack(SYSTEM_ID, COMPONENT_ID, &mavlinkMessage, 255, MAV_BATTERY_FUNCTION_UNKNOWN, MAV_BATTERY_TYPE_LIPO,
                	bmData.temperature, bmData.voltages, bmData.currentBattery, bmData.currentConsumed, bmData.energyConsumed, bmData.batteryRemaining,
					bmData.timeRemaining, bmData.chargeState, {}, 0, 0);
                break;
            }

            case TMMessage_t::RAW_IMU_DATA: {
                auto rawImuData = tmqMessage.tmMessageData.rawImuData;
                mavlink_msg_raw_imu_pack(SYSTEM_ID, COMPONENT_ID, &mavlinkMessage, tmqMessage.timeBootMs, rawImuData.xacc, rawImuData.yacc, rawImuData.zacc, rawImuData.xgyro, rawImuData.ygyro, rawImuData.zgyro, rawImuData.xmag, rawImuData.ymag, rawImuData.zmag, rawImuData.id, rawImuData.temperature);
                break;
            }

            case TMMessage_t::ATTITUDE_DATA: {
                auto attitudeData = tmqMessage.tmMessageData.attitudeData;
                mavlink_msg_attitude_pack(SYSTEM_ID, COMPONENT_ID, &mavlinkMessage, tmqMessage.timeBootMs, attitudeData.roll, attitudeData.pitch, attitudeData.yaw, attitudeData.rollspeed, attitudeData.pitchspeed, attitudeData.yawspeed);
                break;
            }

            default: {
                continue;
            }
        }
        
        messageBuffer->push(&mavlinkMessage);
    }

	if (rc) {
		auto rcData = rcMsg.tmMessageData.rcData;
		mavlink_message_t mavlinkMessage = {0};
		mavlink_msg_rc_channels_pack(SYSTEM_ID, COMPONENT_ID, &mavlinkMessage, rcMsg.timeBootMs, 6,
			rcData.roll, rcData.pitch, rcData.throttle, rcData.yaw, rcData.arm, rcData.flapAngle,  // Channel arrangement from system manager
			UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX,  UINT16_MAX,  UINT16_MAX, UINT16_MAX, UINT8_MAX);
		if (mavlinkMessage.len == 0) {
			return;
		}
		messageBuffer->push(&mavlinkMessage);
	}
}

void TelemetryManager::transmit() {
    static uint8_t transmitBuffer[TM_MAX_TRANSMISSION_BYTES];
    mavlink_message_t msgToTx{};
    uint16_t txBufIdx = 0;

    // Transmit overflow first if it exists
    if (overflowMsgPending) {
        const uint16_t MSG_LEN = mavlink_msg_to_send_buffer(transmitBuffer + txBufIdx, &overflowBuf);
        txBufIdx += MSG_LEN;
        overflowMsgPending = false;
    }

    if (messageBuffer->count() == 0 && txBufIdx == 0) {
        // Nothing to transmit
        return;
    }

    while (messageBuffer->count() > 0 && txBufIdx < TM_MAX_TRANSMISSION_BYTES) {
        messageBuffer->get(&msgToTx);
        const uint16_t MSG_LEN = mavlink_msg_to_send_buffer(transmitBuffer + txBufIdx, &msgToTx);

        if (txBufIdx + MSG_LEN > TM_MAX_TRANSMISSION_BYTES) {
            // Store overflow message for next transmission
            overflowBuf = msgToTx;
            overflowMsgPending = true;
            break;
        }

        txBufIdx += MSG_LEN;
    }

    rfdDriver->transmit(transmitBuffer, txBufIdx);
}

void TelemetryManager::reconstructMsg() {
    uint8_t rxBuffer[RX_BUFFER_LEN];

    const uint16_t RECEIVED_BYTES = rfdDriver->receive(rxBuffer, sizeof(rxBuffer));

    // Use mavlink_parse_char to process one byte at a time
    for (uint16_t i = 0; i < RECEIVED_BYTES; ++i) {
        if (mavlink_parse_char(0, rxBuffer[i], &message, &status)) {
            handleRxMsg(message);
            message = {};
        }
    }
}

void TelemetryManager::handleRxMsg(const mavlink_message_t &msg) {
    switch (msg.msgid) {
        case MAVLINK_MSG_ID_PARAM_SET: {
            float valueToSet;
            char paramToSet[MAVLINK_MAX_IDENTIFIER_LEN] = {};
            uint8_t valueType;
            valueToSet = mavlink_msg_param_set_get_param_value(&msg);
            valueType = mavlink_msg_param_set_get_param_type(&msg);

            if(paramToSet[0] == 'A'){ // Would prefer to do this using an ENUM LUT but if this is the only param being set its whatever
                RCMotorControlMessage_t armDisarmMsg{};
                armDisarmMsg.arm = valueToSet;
                amQueueDriver->push(&armDisarmMsg);
            }
            mavlink_message_t response = {};
            mavlink_msg_param_value_pack(SYSTEM_ID, COMPONENT_ID, &response, paramToSet, valueToSet, valueType, 1, 0);
            messageBuffer->push(&response);
            break;
        }

        default: {
            break;
        }
    }
}
