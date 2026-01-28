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
    IMessageQueue<TMMessage_t> *tmRxQueue,
    IMessageQueue<TMSMMessage_t> *tmTxQueue,
    IMessageQueue<RCMotorControlMessage_t> *amQueueDriver,
    IMessageQueue<mavlink_message_t> *mavlinkTxQueue
) :
    systemUtilsDriver(systemUtilsDriver),
    rfdDriver(rfdDriver),
    tmRxQueue(tmRxQueue),
    tmTxQueue(tmTxQueue),
    amQueueDriver(amQueueDriver),
    mavlinkTxQueue(mavlinkTxQueue),
    overflowMsgPending(false) {}

TelemetryManager::~TelemetryManager() = default;

void TelemetryManager::tmUpdate() {
    reconstructMsg();
    processMsgQueue();
    transmit();
}

// TODO: HERE UNTIL ALL PARAM LIST HAS BEEN SENT, INGORE OTHERS EXCEPT HEARTBEAT
// WHEN THE REQUEST FOR PARAMS IS RECEIVED, SET A BOOLEAN TO IGNORE
void TelemetryManager::processMsgQueue() {
    uint16_t count = tmRxQueue->count();
    TMMessage rcMsg = {};
    bool rc = false;
	while (count-- > 0) {
        mavlink_message_t mavlinkMessage = {0};
        TMMessage_t tmqMessage = {};
        tmRxQueue->get(&tmqMessage);

        switch (tmqMessage.dataType) {
            case TMMessage_t::HEARTBEAT_DATA: {
                auto heartbeatData = tmqMessage.tmMessageData.heartbeatData;
                mavlink_msg_heartbeat_pack(SYSTEM_ID, COMPONENT_ID, &mavlinkMessage, MAV_TYPE_FIXED_WING, MAV_AUTOPILOT_INVALID,
                	heartbeatData.baseMode, heartbeatData.customMode, heartbeatData.systemStatus);
                break;
            }

            case TMMessage_t::GPOS_DATA: {
                auto gposData = tmqMessage.tmMessageData.gposData;
                mavlink_msg_global_position_int_pack(SYSTEM_ID, COMPONENT_ID, &mavlinkMessage, tmqMessage.timeBootMs,
                	gposData.lat, gposData.lon, gposData.alt, gposData.relativeAlt, gposData.vx, gposData.vy, gposData.vz, gposData.hdg);
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

            case TMMessage_t::PARAM_VALUE_DATA: {
                auto paramData = tmqMessage.tmMessageData.paramData;
                mavlink_msg_param_value_pack(SYSTEM_ID, COMPONENT_ID, &mavlinkMessage,
                	paramData.key, paramData.value, paramData.type, paramData.count, paramData.index);
                break;
            }

            default: {
                continue;
            }
        }
        // Check to see if all params have been sent and set flag to false to allow other messages
        if (tmqMessage.dataType == TMMessage_t::PARAM_VALUE_DATA && tmqMessage.tmMessageData.paramData.index + 1 >= tmqMessage.tmMessageData.paramData.count) {
            transmittingParams = false;
        }

        // Allow other messages if not transmitting params, otherwise only allow param and heartbeat messages
        if (!transmittingParams || (transmittingParams && (tmqMessage.dataType == TMMessage_t::PARAM_VALUE_DATA || tmqMessage.dataType == TMMessage_t::HEARTBEAT_DATA))) {
            mavlinkTxQueue->push(&mavlinkMessage);
        }
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
		mavlinkTxQueue->push(&mavlinkMessage);
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

    if (mavlinkTxQueue->count() == 0 && txBufIdx == 0) {
        // Nothing to transmit
        return;
    }

    while (mavlinkTxQueue->count() > 0 && txBufIdx < TM_MAX_TRANSMISSION_BYTES) {
        mavlinkTxQueue->get(&msgToTx);
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
        case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
            transmittingParams = true; // Set flag to ignore other messages except heartbeat and param value until all params sent
            TMSMMessage_t smMsg = requestPack(systemUtilsDriver->getCurrentTimestampMs(), TMSMRequest::REQUEST_PARAMS);
            tmTxQueue->push(&smMsg);
        }

        case MAVLINK_MSG_ID_PARAM_SET: {
            float valueToSet;
            char paramToSet[MAVLINK_MAX_IDENTIFIER_LEN] = {};
            valueToSet = mavlink_msg_param_set_get_param_value(&msg);
            mavlink_msg_param_set_get_param_id(&msg, paramToSet);

            // Send param change to system manager which will respond back with updated param value
            TMSMMessage_t smMsg = paramChangePack(systemUtilsDriver->getCurrentTimestampMs(), paramToSet, valueToSet);
            tmTxQueue->push(&smMsg);
        }

        default: {
            break;
        }
    }
}
