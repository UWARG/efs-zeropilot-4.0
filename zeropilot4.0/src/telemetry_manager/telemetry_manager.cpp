#include "telemetry_manager.hpp"

#define SYSTEM_ID 1             // Suggested System ID by Mavlink
#define COMPONENT_ID 1          // Suggested Component ID by MAVLINK

TelemetryManager::TelemetryManager(
    ISystemUtils *systemUtilsDriver,
    ITelemLink *telemLinkDriver,
    IMessageQueue<TMMessage_t> *tmTXQueueDriver,
    IMessageQueue<RCMotorControlMessage_t> *amQueueDriver,
    IMessageQueue<mavlink_message_t> *packedMsgBuffer
) :
    systemUtilsDriver(systemUtilsDriver),
    telemLinkDriver(telemLinkDriver),
    tmTXQueueDriver(tmTXQueueDriver),
    amQueueDriver(amQueueDriver),
    packedMsgBuffer(packedMsgBuffer),
    overflowMsgPending(false) {}

TelemetryManager::~TelemetryManager() = default;

void TelemetryManager::tmUpdate() {
	receive();
    processTXMsgQueue();
    transmit();
}

void TelemetryManager::processTXMsgQueue() {
    uint16_t count = tmTXQueueDriver->count();
    TMMessage rcMsg = {};
    bool rc = false;
	
    while (count-- > 0) {
        mavlink_message_t mavlinkMessage = {0};
        TMMessage_t tmqMessage = {};
        tmTXQueueDriver->get(&tmqMessage);

        switch (tmqMessage.dataType) {
            case TMMessage_t::HEARTBEAT_DATA: {
                auto heartbeatData = tmqMessage.tmMessageData.heartbeatData;
                mavlink_msg_heartbeat_pack(SYSTEM_ID, COMPONENT_ID, &mavlinkMessage, MAV_TYPE_FIXED_WING, MAV_AUTOPILOT_ARDUPILOTMEGA,
                	heartbeatData.baseMode, heartbeatData.customMode, heartbeatData.systemStatus);
                break;
            }

            case TMMessage_t::STATUSTEXT_DATA: {
                auto& statusTextData = tmqMessage.tmMessageData.statusTextData;
                mavlink_msg_statustext_pack(SYSTEM_ID, COMPONENT_ID, &mavlinkMessage, statusTextData.severity, statusTextData.text, statusTextData.id, statusTextData.chunkSeq);
                break;
            }

            case TMMessage_t::GPS_RAW_DATA: {
                auto& g = tmqMessage.tmMessageData.gpsRawData;
                mavlink_msg_gps_raw_int_pack(SYSTEM_ID, COMPONENT_ID, &mavlinkMessage, (uint64_t)tmqMessage.timeBootMs * 1000,
                    g.fixType, g.lat, g.lon, g.alt, g.eph, g.epv, g.vel, g.cog, g.satellitesVisible, g.altEllipsoid, 
                    g.hAcc, g.vAcc, g.velAcc, g.hdgAcc, g.yaw);
                break;
            }

            case TMMessage_t::SERVO_OUTPUT_RAW: {
                auto& s = tmqMessage.tmMessageData.servoOutputRawData;
                mavlink_msg_servo_output_raw_pack(SYSTEM_ID, COMPONENT_ID, &mavlinkMessage, tmqMessage.timeBootMs, s.port,
                    s.servo1Raw, s.servo2Raw, s.servo3Raw, s.servo4Raw, s.servo5Raw, s.servo6Raw, s.servo7Raw, s.servo8Raw,
                    s.servo9Raw, s.servo10Raw, s.servo11Raw, s.servo12Raw, s.servo13Raw, s.servo14Raw, s.servo15Raw, s.servo16Raw);
                break;
            }

            case TMMessage_t::RC_DATA: {
                rcMsg = tmqMessage;
                rc = true;
                continue;
            }

            case TMMessage_t::BATTERY_DATA: {
                auto batteryData = tmqMessage.tmMessageData.batteryData;
                uint32_t faultBitmask =  (batteryData.chargeState == MAV_BATTERY_CHARGE_STATE_CRITICAL) ? MAV_BATTERY_FAULT_DEEP_DISCHARGE : 0;
                mavlink_msg_battery_status_pack(SYSTEM_ID, COMPONENT_ID, &mavlinkMessage, batteryData.batteryId, MAV_BATTERY_FUNCTION_ALL, MAV_BATTERY_TYPE_LIPO,
                	batteryData.temperature, batteryData.voltages, batteryData.currentBattery, batteryData.currentConsumed, batteryData.energyConsumed, 
                    batteryData.batteryRemaining, batteryData.timeRemaining, batteryData.chargeState, {}, 0, faultBitmask);
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
        
        packedMsgBuffer->push(&mavlinkMessage);
    }

	if (rc) {
		auto& rcData = rcMsg.tmMessageData.rcData;
		mavlink_message_t mavlinkMessage = {0};
		mavlink_msg_rc_channels_pack(SYSTEM_ID, COMPONENT_ID, &mavlinkMessage, rcMsg.timeBootMs, rcData.channelCount,
			rcData.channels[0], rcData.channels[1], rcData.channels[2], rcData.channels[3], 
			rcData.channels[4], rcData.channels[5], rcData.channels[6], rcData.channels[7], 
			rcData.channels[8], rcData.channels[9], rcData.channels[10], rcData.channels[11], 
			rcData.channels[12], rcData.channels[13], rcData.channels[14], rcData.channels[15], 
			rcData.channels[16], rcData.channels[17], UINT8_MAX);
		if (mavlinkMessage.len == 0) {
			return;
		}
		packedMsgBuffer->push(&mavlinkMessage);
	}
}

void TelemetryManager::transmit() {
    mavlink_message_t msgToTX{};
    uint16_t txBufIdx = 0;

    // Transmit overflow first if it exists
    if (overflowMsgPending) {
        const uint16_t MSG_LEN = mavlink_msg_to_send_buffer(txBuffer + txBufIdx, &overflowBuf);
        txBufIdx += MSG_LEN;
        overflowMsgPending = false;
    }

    if (packedMsgBuffer->count() == 0 && txBufIdx == 0) {
        // Nothing to transmit
        return;
    }

    while (packedMsgBuffer->count() > 0 && txBufIdx < TM_MAX_TX_BYTES) {
        packedMsgBuffer->get(&msgToTX);
        const uint16_t MSG_LEN = mavlink_msg_to_send_buffer(txBuffer + txBufIdx, &msgToTX);

        if (txBufIdx + MSG_LEN > TM_MAX_TX_BYTES) {
            // Store overflow message for next transmission
            overflowBuf = msgToTX;
            overflowMsgPending = true;
            break;
        }

        txBufIdx += MSG_LEN;
    }

    telemLinkDriver->transmit(txBuffer, txBufIdx);
}

void TelemetryManager::receive() {
    mavlink_message_t msgToRX{};

    const uint16_t RECEIVED_BYTES = telemLinkDriver->receive(rxBuffer, sizeof(rxBuffer));

    // Use mavlink_parse_char to process one byte at a time
    for (uint16_t i = 0; i < RECEIVED_BYTES; ++i) {
        if (mavlink_parse_char(0, rxBuffer[i], &msgToRX, &status)) {
            processRxMsg(msgToRX);
            msgToRX = {};
        }
    }
}

void TelemetryManager::processRxMsg(const mavlink_message_t &msg) {
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
            packedMsgBuffer->push(&response);
            break;
        }

        default: {
            break;
        }
    }
}
