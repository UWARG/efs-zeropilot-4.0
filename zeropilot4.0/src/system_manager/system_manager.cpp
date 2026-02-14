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
        smSchedulingCounter(0),
        batteryCount(1),
        batteryArray(new batteryData_t[batteryCount]){
            for (size_t i = 0; i < batteryCount; i++){
                batteryArray[i].batteryId = i;
                batteryArray[i].chargeState = MAV_BATTERY_CHARGE_STATE_UNDEFINED;
                batteryArray[i].batteryLowCounterMs = 0;
                batteryArray[i].batteryCritcounterMs = 0;
            }
    }

SystemManager::~SystemManager(){
    delete[] batteryArray;
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
            loggerDriver->log("RC Reconnected");
            rcConnected = true;
        }
    } else {
        oldDataCount += 1;

        if ((oldDataCount * SM_CONTROL_LOOP_DELAY > SM_RC_TIMEOUT) && rcConnected) {
            loggerDriver->log("RC Disconnected");
            rcConnected = false;
        }
    }

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

    // Send Battery Management data to TM and monitor battery state
    MAV_BATTERY_CHARGE_STATE currentBatteryState;
    for (int i = 0; i < batteryCount; i++) {
        if (pmDriver->readData(&(batteryArray[i].pmData))) {          
            currentBatteryState = batteryArray[i].chargeState;

            // Normal battery
            if (batteryArray[i].pmData.busVoltage >= BATTERY_LOW_VOLTAGE){
                batteryArray[i].chargeState = MAV_BATTERY_CHARGE_STATE_OK;
                batteryArray[i].batteryLowCounterMs = 0;
                batteryArray[i].batteryCritcounterMs = 0;
            }
            
            // Low battery detection
            else if (batteryArray[i].pmData.busVoltage >= BATTERY_CRITICAL_VOLTAGE) {
                batteryArray[i].batteryLowCounterMs += SM_CONTROL_LOOP_DELAY;
                batteryArray[i].batteryCritcounterMs = 0;
                if (batteryArray[i].batteryLowCounterMs >= BATTERY_LOW_TIME_MS) {
                    batteryArray[i].chargeState = MAV_BATTERY_CHARGE_STATE_LOW;
                    sendBMDataToTelemetryManager(batteryArray[i]); //we can either transmit one for each pmDriver OR submit 1 overall
                }
            } 

            // Critical battery detection
            else {
                batteryArray[i].batteryCritcounterMs += SM_CONTROL_LOOP_DELAY;
                batteryArray[i].batteryLowCounterMs = 0;
                if (batteryArray[i].batteryCritcounterMs >= BATTERY_CRITICAL_TIME_MS) {
                    batteryArray[i].chargeState = MAV_BATTERY_CHARGE_STATE_CRITICAL;
                    sendBMDataToTelemetryManager(batteryArray[i]); //we can either transmit one for each pmDriver OR submit 1 overall
                }
            } 
    
            // Logging --> once per transition, checks if the state has yet to be logged and does so 
            if (currentBatteryState != batteryArray[i].chargeState) {
                switch (batteryArray[i].chargeState) {
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

void SystemManager::sendBMDataToTelemetryManager(const batteryData_t batteryData) {   
    float voltages[1] = {batteryData.pmData.busVoltage};        
    TMMessage_t bmDataMsg = bmDataPack(systemUtilsDriver->getCurrentTimestampMs(), batteryData.batteryId, INT16_MAX, voltages, 1, batteryData.pmData.charge, batteryData.pmData.current, batteryData.pmData.energy, -1, 0, batteryData.chargeState);
    tmQueue->push(&bmDataMsg);
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
