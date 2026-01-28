#include "system_manager.hpp"

#define SM_SCHEDULING_RATE_HZ 20
#define SM_TELEMETRY_HEARTBEAT_RATE_HZ 1
#define SM_TELEMETRY_RC_DATA_RATE_HZ 5

SystemManager::SystemManager(
    ISystemUtils *systemUtilsDriver,
    IIndependentWatchdog *iwdgDriver,
    IRCReceiver *rcDriver,
	IPowerModule *pmDriver,
    IMessageQueue<RCMotorControlMessage_t> *amRCQueue,
    IMessageQueue<TMMessage_t> *tmQueue,
    IMessageQueue<TMSMMessage_t> *tmSmQueue,
    IMessageQueue<char[100]> *smLoggerQueue,
    IMessageQueue<ConfigMessage_t> *smConfigRouteQueue[],
    Logger *logger,
    Config *config) :
        systemUtilsDriver(systemUtilsDriver),
        iwdgDriver(iwdgDriver),
        rcDriver(rcDriver),
		pmDriver(pmDriver),
        amRCQueue(amRCQueue),
        tmQueue(tmQueue),
        tmSmQueue(tmSmQueue),
        smLoggerQueue(smLoggerQueue),
        smConfigRouteQueue(),
        logger(logger),
        config(config) {
            for (size_t i = 0; i < static_cast<size_t>(Owner_e::COUNT); ++i) {
                this->smConfigRouteQueue[i] = smConfigRouteQueue[i];
            }
        }

void SystemManager::smUpdate() {
    // Kick the watchdog
    iwdgDriver->refreshWatchdog();

    // Get RC data from the RC receiver and passthrough to AM if new
    static int oldDataCount = 0;
    static bool rcConnected = true;

    RCControl rcData = rcDriver->getRCData();
    if (rcData.isDataNew) {
        oldDataCount = 0;
        sendRCDataToAttitudeManager(rcData);

        if (!rcConnected) {
            logger->log("RC Reconnected");
            rcConnected = true;
        }
    } else {
        oldDataCount += 1;

        if ((oldDataCount * SM_CONTROL_LOOP_DELAY > SM_RC_TIMEOUT) && rcConnected) {
            logger->log("RC Disconnected");
            rcConnected = false;
        }
    }

    // Handle messages from TM
    handleMessagesFromTelemetryManager();

    // Send RC data to TM
    if (smSchedulingCounter % (SM_SCHEDULING_RATE_HZ / SM_TELEMETRY_RC_DATA_RATE_HZ) == 0) {
        sendRCDataToTelemetryManager(rcData);
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
        sendHeartbeatDataToTelemetryManager(baseMode, customMode, systemStatus);
    }

    if (pmDriver) {
		PMData_t pmData;
		bool pmDataValid = pmDriver->readData(&pmData);
		(void)pmDataValid; // TODO: remove when used, this line is to suppress -Wunused-variable
	}

    // Send Param data to TM if there are params left to send and the process has been initiated by a param request which sets paramAmountSent to 0
    if (paramAmountSent >= 0 && paramAmountSent < NUM_KEYS) {
        sendParamDataToTelemetryManager();
    }

    // Log if new messages
    if (smLoggerQueue->count() > 0) {
        sendMessagesToLogger();
    }

    // Increment scheduling counter
    smSchedulingCounter = (smSchedulingCounter + 1) % SM_SCHEDULING_RATE_HZ;
}

void SystemManager::sendRCDataToTelemetryManager(const RCControl &rcData) {
    TMMessage_t rcDataMsg =  rcDataPack(systemUtilsDriver->getCurrentTimestampMs(), rcData.roll, rcData.pitch, rcData.yaw, rcData.throttle, rcData.aux2, rcData.arm);
    tmQueue->push(&rcDataMsg);
}

void SystemManager::sendHeartbeatDataToTelemetryManager(uint8_t baseMode, uint32_t customMode, MAV_STATE systemStatus) {
    TMMessage_t hbDataMsg = heartbeatPack(systemUtilsDriver->getCurrentTimestampMs(), baseMode, customMode, systemStatus);
    tmQueue->push(&hbDataMsg);
}

void SystemManager::sendParamDataToTelemetryManager() {
    char buffer[100];
    snprintf(buffer, sizeof(buffer), "Sending param index %d to TM", paramAmountSent);
    logger->log(buffer);
    Param_t param = config->getParam(paramAmountSent);
    TMMessage_t paramDataMsg = paramDataPack(
        systemUtilsDriver->getCurrentTimestampMs(),
        static_cast<uint16_t>(paramAmountSent),
        static_cast<uint16_t>(NUM_KEYS),
        param
    );
    tmQueue->push(&paramDataMsg);
    paramAmountSent++;
}

void SystemManager::sendRCDataToAttitudeManager(const RCControl &rcData) {
    RCMotorControlMessage_t rcDataMessage;

    rcDataMessage.roll = rcData.roll;
    rcDataMessage.pitch = rcData.pitch;
    rcDataMessage.yaw = rcData.yaw;
    rcDataMessage.throttle = rcData.throttle;
    rcDataMessage.arm = rcData.arm;
    rcDataMessage.flapAngle = rcData.aux2;

    amRCQueue->push(&rcDataMessage);
}

void SystemManager::sendMessagesToLogger() {
    static char messages[16][100];
    int msgCount = 0;

    while (smLoggerQueue->count() > 0) {
        smLoggerQueue->get(&messages[msgCount]);
        msgCount++;
    }

    logger->log(messages, msgCount);
}

void SystemManager::handleMessagesFromTelemetryManager() {
    uint16_t count = tmSmQueue->count();
    while (count-- > 0) {
        TMSMMessage_t msg;
        tmSmQueue->get(&msg);
        switch (msg.dataType) {
            case TMSMMessage_t::PARAM_CHANGE_DATA: {
                logger->log("Param change request received from TM");
                int res = config->writeParamByName(
                    msg.tmSMMessageData.paramChangeData.keyId,
                    msg.tmSMMessageData.paramChangeData.value
                );
                if (res == 0) {
                    logger->log("Param updated from TM");
                    size_t key = config->getParamConfigKey(msg.tmSMMessageData.paramChangeData.keyId);
                    Owner_e owner = config->getParamOwner(key);
                    if (owner != Owner_e::COUNT) {
                        ConfigMessage_t configMsg = {
                            .key = static_cast<size_t>(key),
                            .value = msg.tmSMMessageData.paramChangeData.value
                        };
                        smConfigRouteQueue[static_cast<size_t>(owner)]->push(&configMsg);
                        TMMessage_t paramDataMsg = paramDataPack(systemUtilsDriver->getCurrentTimestampMs(),
                            static_cast<uint16_t>(static_cast<size_t>(key)),
                            static_cast<uint16_t>(NUM_KEYS),
                            config->getParam(key)
                        );
                        tmQueue->push(&paramDataMsg);
                    } else {
                        logger->log("Param owner invalid, cannot route update");
                    }
                } else {
                    logger->log("Failed to write param from TM");
                }
                break;
            }

            case TMSMMessage_t::REQUEST_DATA: {
                if (msg.tmSMMessageData.requestData.requestType == TMSMRequest::REQUEST_PARAMS) {
                    paramAmountSent = 0; // Start sending params from the beginning
                    logger->log("Param request received from TM");
                }
                break;
            }

            default:
                break;
        }
    }
}
