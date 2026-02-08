#include "system_manager.hpp"

#define SM_SCHEDULING_RATE_HZ 20
#define SM_TELEMETRY_HEARTBEAT_RATE_HZ 1
#define SM_TELEMETRY_RC_DATA_RATE_HZ 5

#define BATTERY_LOW_VOLTAGE      10.5f
#define BATTERY_CRITICAL_VOLTAGE 9.8f

#define BATTERY_LOW_TIME_MS      10000
#define BATTERY_CRITICAL_TIME_MS 3000

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
        batteryLow(false),
        batteryCritical(false),
        batteryLowLogged(false),
        batteryCritLogged(false),
        batteryVoltage(0.0),
        batteryCurrent(0.0),
        batteryDataValid(false),
        batteryLowCounterMs(0),
        batteryCritcounterMs(0){}

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
    //Validate pmDriver data and then instantiate voltage and current 
    if (pmDriver) {
		PMData_t pmData;
		bool pmDataValid = pmDriver->readData(&pmData);

        if(pmDataValid){
            batteryVoltage =  pmData.busVoltage;
            batteryCurrent = pmData.current;
            batteryDataValid = true;
        }
        else{
            batteryDataValid = false;
        }
	}

    if (batteryDataValid) {
        //Low battery detection 
        if (batteryVoltage < BATTERY_LOW_VOLTAGE) {
            batteryLowCounterMs += SM_CONTROL_LOOP_DELAY;
            if (batteryLowCounterMs >= BATTERY_LOW_TIME_MS) {
                batteryLow = true;
            }
        } else {
            batteryLowCounterMs = 0;
            batteryLow = false;
            batteryLowLogged = false; 
        }

        //Crtitical battery detection
        if (batteryVoltage < BATTERY_CRITICAL_VOLTAGE) {
            batteryCritcounterMs += SM_CONTROL_LOOP_DELAY;
            if (batteryCritcounterMs >= BATTERY_CRITICAL_TIME_MS) {
                batteryCritical = true;
            }
        } else {
            batteryCritcounterMs = 0;
            batteryCritical = false;
            batteryCritLogged = false;
        }
    }


    //Logging --> once per transition, checks if the state has yet to be logged and does so 
    if(batteryLow &&!batteryLowLogged){
        loggerDriver->log("Battery low");
        //TO DO: Send battery data to tm
        batteryLowLogged = true;
    }

    if(batteryCritical && !batteryCritLogged){
        loggerDriver->log("Battery critical");
        //TO DO: Send battery data to tm
        batteryCritLogged = true;
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


void SystemManager::sendMessagesToLogger() {
    static char messages[16][100];
    int msgCount = 0;

    while (smLoggerQueue->count() > 0) {
        smLoggerQueue->get(&messages[msgCount]);
        msgCount++;
    }

    loggerDriver->log(messages, msgCount);
}
