#include "system_manager.hpp"
#include "flightmode.hpp"

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
        smSchedulingCounter(0),
        oldDataCount(0),
        rcConnected(false),
        batteryData({PMData_t{}, MAV_BATTERY_CHARGE_STATE_UNDEFINED, 0, 0}) {}

void SystemManager::smUpdate() {
    // Kick the watchdog
    iwdgDriver->refreshWatchdog();


    // Get RC data from the RC receiver and passthrough to AM if new
    RCControl rcData = rcDriver->getRCData();
    if (rcData.isDataNew) {
        oldDataCount = 0;
        sendRCDataToAttitudeManager(rcData);

        if (!rcConnected) {
            loggerDriver->log("RC Connected");
            rcConnected = true;
        }
    } else {
        oldDataCount += 1;

        if ((oldDataCount * SM_UPDATE_LOOP_DELAY_MS > SM_RC_TIMEOUT_MS) && rcConnected) {
            loggerDriver->log("RC Disconnected");
            rcConnected = false;
        }
    }

    // Send RC data to TM
    if (smSchedulingCounter % (SM_SCHEDULING_RATE_HZ / SM_TELEMETRY_RC_DATA_RATE_HZ) == 0) {
        sendRCDataToTelemetryManager(rcData);
    }

    // Populate baseMode based on arm state
    uint8_t baseMode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
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

    // Hardcoded to MANUAL for now, should come from RC input in future
    uint32_t customMode = static_cast<uint32_t>(PlaneFlightMode_e::MANUAL);

    // Send Heartbeat data to TM at a 1Hz rate
    if (smSchedulingCounter % (SM_SCHEDULING_RATE_HZ / SM_TELEMETRY_HEARTBEAT_RATE_HZ) == 0) {
        sendHeartbeatDataToTelemetryManager(baseMode, customMode, systemStatus);
    }

    // Monitor Battery State and send Battery Data to TM at a 1Hz rate
    updateBatteryFSM();
    if (smSchedulingCounter % (SM_SCHEDULING_RATE_HZ / SM_TELEMETRY_BATTERY_DATA_RATE_HZ) == 0) {
        sendBatteryDataToTelemetryManager(batteryData);
    }

    // Log if new messages
    if (smLoggerQueue->count() > 0) {
        sendMessagesToLogger();
    }

    // Increment scheduling counter
    smSchedulingCounter = (smSchedulingCounter + 1) % SM_SCHEDULING_RATE_HZ;
}

void SystemManager::updateBatteryFSM() {
    MAV_BATTERY_CHARGE_STATE currentBatteryState;
    if (pmDriver->readData(&(batteryData.pmData))) {          
        currentBatteryState = batteryData.chargeState;

        // Normal battery
        if (batteryData.pmData.busVoltage >= BATTERY_LOW_VOLTAGE){
            batteryData.chargeState = MAV_BATTERY_CHARGE_STATE_OK;
            batteryData.batteryLowCounterMs = 0;
            batteryData.batteryCritcounterMs = 0;
        }
        
        // Low battery detection
        else if (batteryData.pmData.busVoltage >= BATTERY_CRITICAL_VOLTAGE) {
            batteryData.batteryLowCounterMs += SM_UPDATE_LOOP_DELAY_MS;
            batteryData.batteryCritcounterMs = 0;
            if (batteryData.batteryLowCounterMs >= SM_BATTERY_LOW_TIME_MS) {
                batteryData.chargeState = MAV_BATTERY_CHARGE_STATE_LOW;
                sendBatteryDataToTelemetryManager(batteryData);
            }
        } 

        // Critical battery detection
        else {
            batteryData.batteryCritcounterMs += SM_UPDATE_LOOP_DELAY_MS;
            batteryData.batteryLowCounterMs = 0;
            if (batteryData.batteryCritcounterMs >= SM_BATTERY_CRITICAL_TIME_MS) {
                batteryData.chargeState = MAV_BATTERY_CHARGE_STATE_CRITICAL;
                sendBatteryDataToTelemetryManager(batteryData);
            }
        } 

        // Logging --> once per transition, checks if the state has yet to be logged and does so 
        if (currentBatteryState != batteryData.chargeState) {
            switch (batteryData.chargeState) {
                case MAV_BATTERY_CHARGE_STATE_OK:
                    loggerDriver->log("Battery State: OK");
                    break;
                case MAV_BATTERY_CHARGE_STATE_LOW:
                    loggerDriver->log("Battery State: LOW");
                    break;
                case MAV_BATTERY_CHARGE_STATE_CRITICAL:
                    loggerDriver->log("Battery State: CRITICAL");
                    break;
                default:
                    break;
            }
        }    
    }    
}

void SystemManager::sendRCDataToTelemetryManager(const RCControl &rcData) {
    TMMessage_t rcDataMsg =  rcDataPack(systemUtilsDriver->getCurrentTimestampMs(), rcData.roll, rcData.pitch, rcData.yaw, rcData.throttle, rcData.aux2, rcData.arm);
    tmQueue->push(&rcDataMsg);
}

void SystemManager::sendHeartbeatDataToTelemetryManager(uint8_t baseMode, uint32_t customMode, MAV_STATE systemStatus) {
    TMMessage_t hbDataMsg = heartbeatPack(systemUtilsDriver->getCurrentTimestampMs(), baseMode, customMode, systemStatus);
    tmQueue->push(&hbDataMsg);
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

void SystemManager::sendBatteryDataToTelemetryManager(const BatteryData_t &batteryData) {   
    float voltages[1] = {batteryData.pmData.busVoltage};
    TMMessage_t batteryDataMsg = batteryDataPack(systemUtilsDriver->getCurrentTimestampMs(), INT16_MAX, voltages, 1, batteryData.pmData.current, batteryData.pmData.charge, batteryData.pmData.energy, -1, 0, batteryData.chargeState);
    tmQueue->push(&batteryDataMsg);
}

void SystemManager::sendMessagesToLogger() {
    static char messages[16][100];
    int msgCount = 0;

    while (smLoggerQueue->count() > 0) {
        smLoggerQueue->get(&messages[msgCount]);
        msgCount++;
    }

    loggerDriver->log(messages, msgCount);
}
