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

ZP_ERROR_e TelemetryManager::tmUpdate() {
    ZP_RETURN_IF_ERROR(processMsgQueue());
    ZP_RETURN_IF_ERROR(transmit());
}

ZP_ERROR_e TelemetryManager::processMsgQueue() {
    int countVal = 0;
    ZP_RETURN_IF_ERROR(tmQueueDriver->count(&countVal));
    uint16_t count = countVal;
    TMMessage rcMsg = {};
    bool rc = false;
	while (count-- > 0) {
        mavlink_message_t mavlinkMessage = {0};
        TMMessage_t tmqMessage = {};
        ZP_RETURN_IF_ERROR(tmQueueDriver->get(&tmqMessage));

        switch (tmqMessage.dataType) {
            case TMMessage_t::HEARTBEAT_DATA: {
                auto heartbeatData = tmqMessage.tmMessageData.heartbeatData;
                ZP_RETURN_IF_ERROR(mavlink_msg_heartbeat_pack(SYSTEM_ID, COMPONENT_ID, &mavlinkMessage, MAV_TYPE_FIXED_WING, MAV_AUTOPILOT_INVALID,
                	heartbeatData.baseMode, heartbeatData.customMode, heartbeatData.systemStatus));
                break;
            }

            case TMMessage_t::GPOS_DATA: {
                auto gposData = tmqMessage.tmMessageData.gposData;
                ZP_RETURN_IF_ERROR(mavlink_msg_global_position_int_pack(SYSTEM_ID, COMPONENT_ID, &mavlinkMessage, tmqMessage.timeBootMs,
                	gposData.lat, gposData.lon, gposData.alt, gposData.relativeAlt, gposData.vx, gposData.vy, gposData.vz, gposData.hdg));
                break;
            }

            case TMMessage_t::RC_DATA: {
                rcMsg = tmqMessage;
                rc = true;
                continue;
            }

            case TMMessage_t::BM_DATA: {
                auto bmData = tmqMessage.tmMessageData.bmData;
                ZP_RETURN_IF_ERROR(mavlink_msg_battery_status_pack(SYSTEM_ID, COMPONENT_ID, &mavlinkMessage, 255, MAV_BATTERY_FUNCTION_UNKNOWN, MAV_BATTERY_TYPE_LIPO,
                	bmData.temperature, bmData.voltages, bmData.currentBattery, bmData.currentConsumed, bmData.energyConsumed, bmData.batteryRemaining,
					bmData.timeRemaining, bmData.chargeState, {}, 0, 0));
                break;
            }

            case TMMessage_t::RAW_IMU_DATA: {
                auto rawImuData = tmqMessage.tmMessageData.rawImuData;
                ZP_RETURN_IF_ERROR(mavlink_msg_raw_imu_pack(SYSTEM_ID, COMPONENT_ID, &mavlinkMessage, tmqMessage.timeBootMs, rawImuData.xacc, rawImuData.yacc, rawImuData.zacc, rawImuData.xgyro, rawImuData.ygyro, rawImuData.zgyro, rawImuData.xmag, rawImuData.ymag, rawImuData.zmag, rawImuData.id, rawImuData.temperature));
                break;
            }

            case TMMessage_t::ATTITUDE_DATA: {
                auto attitudeData = tmqMessage.tmMessageData.attitudeData;
                ZP_RETURN_IF_ERROR(mavlink_msg_attitude_pack(SYSTEM_ID, COMPONENT_ID, &mavlinkMessage, tmqMessage.timeBootMs, attitudeData.roll, attitudeData.pitch, attitudeData.yaw, attitudeData.rollspeed, attitudeData.pitchspeed, attitudeData.yawspeed));
                break;
            }

            default: {
                continue;
            }
        }
        
        ZP_RETURN_IF_ERROR(messageBuffer->push(&mavlinkMessage));
    }

	if (rc) {
		auto rcData = rcMsg.tmMessageData.rcData;
		mavlink_message_t mavlinkMessage = {0};
		ZP_RETURN_IF_ERROR(mavlink_msg_rc_channels_pack(SYSTEM_ID, COMPONENT_ID, &mavlinkMessage, rcMsg.timeBootMs, 6,
			rcData.roll, rcData.pitch, rcData.throttle, rcData.yaw, rcData.arm, rcData.flapAngle,  // Channel arrangement from system manager
			UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX,  UINT16_MAX,  UINT16_MAX, UINT16_MAX, UINT8_MAX));
		if (mavlinkMessage.len == 0) {
			return;
		}
		ZP_RETURN_IF_ERROR(messageBuffer->push(&mavlinkMessage));
	}
}

ZP_ERROR_e TelemetryManager::transmit() {
    static uint8_t transmitBuffer[TM_MAX_TRANSMISSION_BYTES];
    mavlink_message_t msgToTx{};
    uint16_t txBufIdx = 0;

    // Transmit overflow first if it exists
    if (overflowMsgPending) {
        const uint16_t MSG_LEN = ZP_RETURN_IF_ERROR(mavlink_msg_to_send_buffer(transmitBuffer + txBufIdx, &overflowBuf));
        txBufIdx += MSG_LEN;
        overflowMsgPending = false;
    }

    int msgCount = 0;
    ZP_RETURN_IF_ERROR(messageBuffer->count(&msgCount));
    if (msgCount == 0 && txBufIdx == 0) {
        // Nothing to transmit
        return;
    }

    while (msgCount > 0 && txBufIdx < TM_MAX_TRANSMISSION_BYTES) {
        ZP_RETURN_IF_ERROR(messageBuffer->get(&msgToTx));
        const uint16_t MSG_LEN = ZP_RETURN_IF_ERROR(mavlink_msg_to_send_buffer(transmitBuffer + txBufIdx, &msgToTx));

        if (txBufIdx + MSG_LEN > TM_MAX_TRANSMISSION_BYTES) {
            // Store overflow message for next transmission
            overflowBuf = msgToTx;
            overflowMsgPending = true;
            break;
        }

        txBufIdx += MSG_LEN;
        ZP_RETURN_IF_ERROR(messageBuffer->count(&msgCount));
    }

    ZP_RETURN_IF_ERROR(rfdDriver->transmit(transmitBuffer, txBufIdx));
}

ZP_ERROR_e TelemetryManager::reconstructMsg() {
    uint8_t rxBuffer[RX_BUFFER_LEN];

    uint16_t receivedBytes = 0;
    ZP_RETURN_IF_ERROR(rfdDriver->receive(&receivedBytes, rxBuffer, sizeof(rxBuffer)));

    // Use mavlink_parse_char to process one byte at a time
    for (uint16_t i = 0; i < receivedBytes; ++i) {
        if (ZP_RETURN_IF_ERROR(mavlink_parse_char(0, rxBuffer[i], &message, &status))) {
            ZP_RETURN_IF_ERROR(handleRxMsg(message));
            message = {};
        }
    }
}

ZP_ERROR_e TelemetryManager::handleRxMsg(const mavlink_message_t &msg) {
    switch (msg.msgid) {
        case MAVLINK_MSG_ID_PARAM_SET: {
            float valueToSet;
            char paramToSet[MAVLINK_MAX_IDENTIFIER_LEN] = {};
            uint8_t valueType;
            ZP_RETURN_IF_ERROR(mavlink_msg_param_set_get_param_value(&msg, &valueToSet));
            ZP_RETURN_IF_ERROR(mavlink_msg_param_set_get_param_type(&msg, &valueType));

            if(paramToSet[0] == 'A'){ // Would prefer to do this using an ENUM LUT but if this is the only param being set its whatever
                RCMotorControlMessage_t armDisarmMsg{};
                armDisarmMsg.arm = valueToSet;
                ZP_RETURN_IF_ERROR(amQueueDriver->push(&armDisarmMsg));
            }
            mavlink_message_t response = {};
            mavlink_msg_param_value_pack(SYSTEM_ID, COMPONENT_ID, &response, paramToSet, valueToSet, valueType, 1, 0);
            ZP_RETURN_IF_ERROR(messageBuffer->push(&response));
            break;
        }

        default: {
            break;
        }
    }
}
