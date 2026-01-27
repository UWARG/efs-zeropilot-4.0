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
    ZP_ERROR_e err;

    // Kick the watchdog
    err = iwdgDriver->refreshWatchdog();
    if (err != ZP_ERROR_OK) {
        return err;
    }

    // Get RC data from the RC receiver and passthrough to AM if new
    static int oldDataCount = 0;
    static bool rcConnected = true;

    RCControl rcData;
    err = rcDriver->getRCData(&rcData);
    if (err != ZP_ERROR_OK) {
        return err;
    }

    if (rcData.isDataNew) {
        oldDataCount = 0;
        err = sendRCDataToAttitudeManager(rcData);
        if (err != ZP_ERROR_OK) {
            return err;
        }

        if (!rcConnected) {
            err = loggerDriver->log("RC Reconnected");
            if (err != ZP_ERROR_OK) {
                return err;
            }
            rcConnected = true;
        }
    } else {
        oldDataCount += 1;

        if ((oldDataCount * SM_MAIN_DELAY > 500) && rcConnected) {
            err = loggerDriver->log("RC Disconnected");
            if (err != ZP_ERROR_OK) {
                return err;
            }
            rcConnected = false;
        }
    }

    // Send RC data to TM
    if (smSchedulingCounter % (SM_SCHEDULING_RATE_HZ / SM_TELEMETRY_RC_DATA_RATE_HZ) == 0) {
        err = sendRCDataToTelemetryManager(rcData);
        if (err != ZP_ERROR_OK) {
            return err;
        }
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
        err = sendHeartbeatDataToTelemetryManager(baseMode, customMode, systemStatus);
         if (err != ZP_ERROR_OK) {
            return err;
        }
    }

    if (pmDriver) {
		PMData_t pmData;
		bool pmDataValid = pmDriver->readData(&pmData);
		(void)pmDataValid; // TODO: remove when used, this line is to suppress -Wunused-variable
        if (err != ZP_ERROR_OK) {
            return err;
        }
	}

    // Log if new messages
    int msgCount = 0;
    err = smLoggerQueue->count(&msgCount);
    if (err != ZP_ERROR_OK) {
        return err;
    }

    if (msgCount > 0) {
        err = sendMessagesToLogger();
        if (err != ZP_ERROR_OK) {
            return err;
        }
    }

    // Increment scheduling counter
    smSchedulingCounter = (smSchedulingCounter + 1) % SM_SCHEDULING_RATE_HZ;

    return ZP_ERROR_OK;
}

ZP_ERROR_e SystemManager::sendRCDataToTelemetryManager(const RCControl &rcData) {
    ZP_ERROR_e err;
    uint32_t timestamp;

    err = systemUtilsDriver->getCurrentTimestampMs(&timestamp);
    if (err != ZP_ERROR_OK) {
        return err;
    }

    TMMessage_t rcDataMsg = rcDataPack(timestamp, rcData.roll, rcData.pitch, rcData.yaw, rcData.throttle, rcData.aux2, rcData.arm);

    err = tmQueue->push(&rcDataMsg);
    if (err != ZP_ERROR_OK) {
        return err;
    }

    return ZP_ERROR_OK;
}

ZP_ERROR_e SystemManager::sendHeartbeatDataToTelemetryManager(uint8_t baseMode, uint32_t customMode, MAV_STATE systemStatus) {
    ZP_ERROR_e err;
    uint32_t timestamp;

    err = systemUtilsDriver->getCurrentTimestampMs(&timestamp);
    if (err != ZP_ERROR_OK) {
        return err;
    }

    TMMessage_t hbDataMsg = heartbeatPack(timestamp, baseMode, customMode, systemStatus);

    err = tmQueue->push(&hbDataMsg);
    if (err != ZP_ERROR_OK) {
        return err;
    }

    return ZP_ERROR_OK;
}

ZP_ERROR_e SystemManager::sendRCDataToAttitudeManager(const RCControl &rcData) {
    ZP_ERROR_e err;
    RCMotorControlMessage_t rcDataMessage;

    rcDataMessage.roll = rcData.roll;
    rcDataMessage.pitch = rcData.pitch;
    rcDataMessage.yaw = rcData.yaw;
    rcDataMessage.throttle = rcData.throttle;
    rcDataMessage.arm = rcData.arm;
    rcDataMessage.flapAngle = rcData.aux2;

    err = amRCQueue->push(&rcDataMessage);
    if (err != ZP_ERROR_OK) {
        return err;
    }

    return ZP_ERROR_OK;
}

ZP_ERROR_e SystemManager::sendMessagesToLogger() {
    ZP_ERROR_e err;
    static char messages[16][100];
    int msgCount = 0;
    int queueCount = 0;

    err = smLoggerQueue->count(&queueCount);
    if (err != ZP_ERROR_OK) {
        return err;
    }

    while (queueCount > 0) {
        err = smLoggerQueue->get(&messages[msgCount]);
        if (err != ZP_ERROR_OK) {
            return err;
        }
        msgCount++;

        err = smLoggerQueue->count(&queueCount);
        if (err != ZP_ERROR_OK) {
            return err;
        }
    }

    err = loggerDriver->log(messages, msgCount);
    if (err != ZP_ERROR_OK) {
        return err;
    }

    return ZP_ERROR_OK;
}
