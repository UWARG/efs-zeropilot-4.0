#include "system_manager.hpp"

#define SM_SCHEDULING_RATE_HZ 20
#define SM_TELEMETRY_HEARTBEAT_RATE_HZ 1
#define SM_TELEMETRY_RC_DATA_RATE_HZ 5

SystemManager::SystemManager(
    ISystemUtils *systemUtilsDriver,
    IIndependentWatchdog *iwdgDriver,
    ILogger *loggerDriver,
    IRCReceiver *rcDriver,
	IPowerModule *pmDriver,
    IMessageQueue<RCMotorControlMessage_t> *amRCQueue,
    IMessageQueue<TMMessage_t> *tmQueue,
    IMessageQueue<char[100]> *smLoggerQueue) :
        systemUtilsDriver(systemUtilsDriver),
        iwdgDriver(iwdgDriver),
        loggerDriver(loggerDriver),
        rcDriver(rcDriver),
		pmDriver(pmDriver),
        amRCQueue(amRCQueue),
        tmQueue(tmQueue),
        smLoggerQueue(smLoggerQueue),
        smSchedulingCounter(0) {}

ZP_ERROR_e SystemManager::smUpdate() {
    // Kick the watchdog
    ZP_RETURN_IF_ERROR(iwdgDriver->refreshWatchdog());

    // Get RC data from the RC receiver and passthrough to AM if new
    static int oldDataCount = 0;
    static bool rcConnected = true;

    RCControl rcData;
    ZP_RETURN_IF_ERROR(rcDriver->getRCData(&rcData));

    if (rcData.isDataNew) {
        oldDataCount = 0;
        ZP_RETURN_IF_ERROR(sendRCDataToAttitudeManager(rcData));

        if (!rcConnected) {
            ZP_RETURN_IF_ERROR(loggerDriver->log("RC Reconnected"));
            rcConnected = true;
        }
    } else {
        oldDataCount += 1;

        if ((oldDataCount * SM_MAIN_DELAY > 500) && rcConnected) {
            ZP_RETURN_IF_ERROR(loggerDriver->log("RC Disconnected"));
            rcConnected = false;
        }
    }

    // Send RC data to TM
    if (smSchedulingCounter % (SM_SCHEDULING_RATE_HZ / SM_TELEMETRY_RC_DATA_RATE_HZ) == 0) {
        ZP_RETURN_IF_ERROR(sendRCDataToTelemetryManager(rcData));
    }

    // Populate baseMode based on arm state
    uint8_t baseMode = MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
    if (rcData.arm) {
        baseMode |= MAV_MODE_FLAG_SAFETY_ARMED;
    }

    // Determine system status based on RC connection and arm state
    MAV_STATE systemStatus = MAV_STATE_ACTIVE;
    if (!rcConnected) {
        systemStatus = MAV_STATE_CRITICAL;
    } else if (!rcData.arm) {
        systemStatus = MAV_STATE_STANDBY;
    }

    // Custom mode not used, set to 0
    uint32_t customMode = 0;

    // Send Heartbeat data to TM at a 1Hz rate
    if (smSchedulingCounter % (SM_SCHEDULING_RATE_HZ / SM_TELEMETRY_HEARTBEAT_RATE_HZ) == 0) {
        ZP_RETURN_IF_ERROR(sendHeartbeatDataToTelemetryManager(baseMode, customMode, systemStatus));
    }

    if (pmDriver) {
		PMData_t pmData;
		ZP_RETURN_IF_ERROR(pmDriver->readData(&pmData));
		(void)pmDataValid; // TODO: remove when used, this line is to suppress -Wunused-variable
        
	}

    // Log if new messages
    int msgCount = 0;
    ZP_RETURN_IF_ERROR(smLoggerQueue->count(&msgCount));

    if (msgCount > 0) {
        ZP_RETURN_IF_ERROR(sendMessagesToLogger());
    }

    // Increment scheduling counter
    smSchedulingCounter = (smSchedulingCounter + 1) % SM_SCHEDULING_RATE_HZ;

    return ZP_ERROR_OK;
}

ZP_ERROR_e SystemManager::sendRCDataToTelemetryManager(const RCControl &rcData) {
    uint32_t timestamp;

    ZP_RETURN_IF_ERROR(systemUtilsDriver->getCurrentTimestampMs(&timestamp));

    TMMessage_t rcDataMsg = rcDataPack(timestamp, rcData.roll, rcData.pitch, rcData.yaw, rcData.throttle, rcData.aux2, rcData.arm);

    ZP_RETURN_IF_ERROR(tmQueue->push(&rcDataMsg));

    return ZP_ERROR_OK;
}

ZP_ERROR_e SystemManager::sendHeartbeatDataToTelemetryManager(uint8_t baseMode, uint32_t customMode, MAV_STATE systemStatus) {
    uint32_t timestamp;

    ZP_RETURN_IF_ERROR(systemUtilsDriver->getCurrentTimestampMs(&timestamp));

    TMMessage_t hbDataMsg = heartbeatPack(timestamp, baseMode, customMode, systemStatus);

    ZP_RETURN_IF_ERROR(tmQueue->push(&hbDataMsg));

    return ZP_ERROR_OK;
}

ZP_ERROR_e SystemManager::sendRCDataToAttitudeManager(const RCControl &rcData) {
    RCMotorControlMessage_t rcDataMessage;

    rcDataMessage.roll = rcData.roll;
    rcDataMessage.pitch = rcData.pitch;
    rcDataMessage.yaw = rcData.yaw;
    rcDataMessage.throttle = rcData.throttle;
    rcDataMessage.arm = rcData.arm;
    rcDataMessage.flapAngle = rcData.aux2;

    ZP_RETURN_IF_ERROR(amRCQueue->push(&rcDataMessage));

    return ZP_ERROR_OK;
}

ZP_ERROR_e SystemManager::sendMessagesToLogger() {
    static char messages[16][100];
    int msgCount = 0;
    int queueCount = 0;

    ZP_RETURN_IF_ERROR(smLoggerQueue->count(&queueCount));

    while (queueCount > 0) {
        ZP_RETURN_IF_ERROR(smLoggerQueue->get(&messages[msgCount]));
        msgCount++;

        ZP_RETURN_IF_ERROR(smLoggerQueue->count(&queueCount));
    }

    ZP_RETURN_IF_ERROR(loggerDriver->log(messages, msgCount));

    return ZP_ERROR_OK;
}
