#include "system_manager.hpp"
#include "zp_params.hpp"
#include "flightmode.hpp"
#include "soc_estimation.hpp"
#include "attitude_manager.hpp"
#include "telemetry_manager.hpp"

#define LOG_TIMING 0

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
        flightModes{},
        oldDataCount(0),
        rcConnected(false),
        batteryData({PMData_t{}, MAV_BATTERY_CHARGE_STATE_OK, 0, 0}),
        socEstimator(batteryData),
        profilerId(0),
        paramSetup(this)
{
    paramSetup.loadAllParams();
    paramSetup.bindAllParamCallbacks();
    systemUtilsDriver->profilerRegister("SM", &profilerId);
}

void SystemManager::smUpdate() {

    systemUtilsDriver->profilerBegin(profilerId);

    // Kick the watchdog
    iwdgDriver->refreshWatchdog();


    // Get RC data from the RC receiver and passthrough to AM if new
    RCControl rcData = rcDriver->getRCData();
    if (rcData.isDataNew) {
        oldDataCount = 0;
        sendRCDataToAttitudeManager(rcData);

        if (!rcConnected) {
            sendStatusTextToTelemetryManager(MAV_SEVERITY_INFO, "RC Connected");
            // loggerDriver->log("RC Connected"); (TODO: Uncomment after rearchitecture)
            rcConnected = true;
        }
    } else {
        oldDataCount += 1;

        if ((oldDataCount * SM_UPDATE_LOOP_DELAY_MS > (ZP_PARAM::get(ZP_PARAM_ID::RC_FS_TIMEOUT) * 1000)) && rcConnected) {
            sendStatusTextToTelemetryManager(MAV_SEVERITY_CRITICAL, "RC Disconnected");
            // loggerDriver->log("RC Disconnected"); (TODO: Uncomment after rearchitecture)
            rcConnected = false;
        }
    }

    // Send RC data to TM
    if (smSchedulingCounter % (SM_SCHEDULING_RATE_HZ / SM_TELEMETRY_RC_DATA_RATE_HZ) == 0) {
        sendRCDataToTelemetryManager(rcData);
    }

    // Set armed status based on SM_RC_ARM_THRESHOLD
    bool armed = rcData.arm > SM_RC_ARM_THRESHOLD;

    // Populate baseMode based on arm state
    uint8_t baseMode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    if (armed) {
        baseMode |= MAV_MODE_FLAG_SAFETY_ARMED;
    }

    // Determine system status based on RC connection and arm state
    MAV_STATE systemStatus = MAV_STATE_ACTIVE;
    if (!rcConnected) {
        systemStatus = MAV_STATE_CRITICAL;
    } else if (!armed) {
        systemStatus = MAV_STATE_STANDBY;
    }

    // Decode flight mode from raw value and include in custom mode for HEARTBEAT telemetry
    PlaneFlightMode_e flightMode = decodeRawFlightMode(rcData.fltModeRaw);
    uint32_t customMode = static_cast<uint32_t>(flightMode);

    // Send Heartbeat data to TM at a 1Hz rate
    if (smSchedulingCounter % (SM_SCHEDULING_RATE_HZ / SM_TELEMETRY_HEARTBEAT_RATE_HZ) == 0) {
        sendHeartbeatDataToTelemetryManager(baseMode, customMode, systemStatus);
    }

    // Monitor Battery State and send Battery Data to TM at a 1Hz rate
    if (updateBatteryFSM()) {
        socEstimator.calcStateOfCharge(batteryData, SOC_CHARGE_DISCHARGE_MODE);
        if (smSchedulingCounter % (SM_SCHEDULING_RATE_HZ / SM_TELEMETRY_BATTERY_DATA_RATE_HZ) == 0) {
            sendBatteryDataToTelemetryManager(batteryData, 0);
        }
    }

    // Log if new messages
    if (smLoggerQueue->count() > 0) {
        sendMessagesToLogger();
    }

    // Send profiler stats at 1Hz
    if (smSchedulingCounter % (SM_SCHEDULING_RATE_HZ / SM_TELEMETRY_HEARTBEAT_RATE_HZ) == 0) {
        uint8_t count = 0;
        systemUtilsDriver->profilerGetAll(profiles, &count);

        for (uint8_t i = 0; i < count; i++) {
            if (strcmp(profiles[i].name, "SM") == 0) {
                if (profiles[i].deltaExec >= (SM_UPDATE_LOOP_DELAY_MS * 1000)) {
                    sendStatusTextToTelemetryManager(MAV_SEVERITY_CRITICAL, "SM execution time exceeding scheduled rate");
                } else if (profiles[i].deltaExec >= 0.8f * (SM_UPDATE_LOOP_DELAY_MS * 1000)) {
                    sendStatusTextToTelemetryManager(MAV_SEVERITY_WARNING, "SM execution time about to exceed scheduled rate");
                }
            } else if (strcmp(profiles[i].name, "AM") == 0) {
                if (profiles[i].deltaExec >= (AM_UPDATE_LOOP_DELAY_MS * 1000)) {
                    sendStatusTextToTelemetryManager(MAV_SEVERITY_CRITICAL, "AM execution time exceeding scheduled rate");
                } else if (profiles[i].deltaExec >= 0.8f * (AM_UPDATE_LOOP_DELAY_MS * 1000)) {
                    sendStatusTextToTelemetryManager(MAV_SEVERITY_WARNING, "AM execution time about to exceed scheduled rate");
                }
            } else if (strcmp(profiles[i].name, "TM") == 0) {
                if (profiles[i].deltaExec >= (TM_UPDATE_LOOP_DELAY_MS * 1000)) {
                    sendStatusTextToTelemetryManager(MAV_SEVERITY_CRITICAL, "TM execution time exceeding scheduled rate");
                } else if (profiles[i].deltaExec >= 0.8f * (TM_UPDATE_LOOP_DELAY_MS * 1000)) {
                    sendStatusTextToTelemetryManager(MAV_SEVERITY_WARNING, "TM execution time about to exceed scheduled rate");
                }
            }
            #if LOG_TIMING
            snprintf((char*)profilerBuf, sizeof(profilerBuf), "%-12s %lu us      %lu hz", profiles[i].name, profiles[i].deltaExec, profiles[i].deltaPeriod);
            sendStatusTextToTelemetryManager(MAV_SEVERITY_INFO, (char*)profilerBuf);
            #endif
        }
        #if LOG_TIMING
        sendStatusTextToTelemetryManager(MAV_SEVERITY_INFO, "-------TASK TIMINGS-------");
        #endif
    }

    // Increment scheduling counter
    smSchedulingCounter = (smSchedulingCounter + 1) % SM_SCHEDULING_RATE_HZ;

    systemUtilsDriver->profilerEnd(profilerId);
}

bool SystemManager::updateBatteryFSM() {
    batteryData.isValid = false;         
    MAV_BATTERY_CHARGE_STATE currentBatteryState;

    if (!pmDriver->readData(&batteryData.pmData)) return false; 
    batteryData.isValid = true;         
    currentBatteryState = batteryData.chargeState;

    if (batteryData.pmData.busVoltage >= ZP_PARAM::get(ZP_PARAM_ID::BATT_LOW_VOLT)) {
        // Normal battery
        batteryData.chargeState = MAV_BATTERY_CHARGE_STATE_OK;
        batteryData.batteryLowCounterMs = 0;
        batteryData.batteryCritcounterMs = 0;
    } else if (batteryData.pmData.busVoltage >= ZP_PARAM::get(ZP_PARAM_ID::BATT_CRT_VOLT)) {
        // Low battery detection
        batteryData.batteryLowCounterMs += SM_UPDATE_LOOP_DELAY_MS;
        batteryData.batteryCritcounterMs = 0;
        uint32_t battLowTimeMs = ZP_PARAM::get(ZP_PARAM_ID::BATT_LOW_TIMER) * 1000;
        if (battLowTimeMs > 0 && batteryData.batteryLowCounterMs >= battLowTimeMs) {
            batteryData.chargeState = MAV_BATTERY_CHARGE_STATE_LOW;
        }
    } else {
        // Critical battery detection
        batteryData.batteryCritcounterMs += SM_UPDATE_LOOP_DELAY_MS;
        batteryData.batteryLowCounterMs = 0;
        uint32_t battLowTimeMs = ZP_PARAM::get(ZP_PARAM_ID::BATT_LOW_TIMER) * 1000;
        if (battLowTimeMs > 0 && batteryData.batteryCritcounterMs >= battLowTimeMs) {
            batteryData.chargeState = MAV_BATTERY_CHARGE_STATE_CRITICAL;
        }
    }

    // Logging --> once per transition, checks if the state has yet to be logged and does so 
    if (currentBatteryState != batteryData.chargeState) {
        switch (batteryData.chargeState) {
            case MAV_BATTERY_CHARGE_STATE_OK:
                sendStatusTextToTelemetryManager(MAV_SEVERITY_INFO, "Battery State: OK");
                // loggerDriver->log("Battery State: OK"); (TODO: Uncomment after rearchitecture)
                break;
            case MAV_BATTERY_CHARGE_STATE_LOW:
                sendStatusTextToTelemetryManager(MAV_SEVERITY_WARNING, "Battery State: LOW");
                // loggerDriver->log("Battery State: LOW"); (TODO: Uncomment after rearchitecture)
                break;
            case MAV_BATTERY_CHARGE_STATE_CRITICAL:
                sendStatusTextToTelemetryManager(MAV_SEVERITY_CRITICAL, "Battery State: CRITICAL");
                // loggerDriver->log("Battery State: CRITICAL"); (TODO: Uncomment after rearchitecture)
                break;
            default:
                break;
        }
    }

    return true;
}

void SystemManager::sendRCDataToTelemetryManager(const RCControl &rcData) {
    TMMessage_t rcDataMsg = rcDataPack(systemUtilsDriver->getCurrentTimestampMs(), rcData.controlSignals, INPUT_CHANNELS);
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
    rcDataMessage.arm = rcData.arm > SM_RC_ARM_THRESHOLD;
    rcDataMessage.flapAngle = rcData.aux2;
    rcDataMessage.flightMode = decodeRawFlightMode(rcData.fltModeRaw);

    amRCQueue->push(&rcDataMessage);
}

void SystemManager::sendBatteryDataToTelemetryManager(const BatteryData_t &batteryData, const uint8_t BATTERY_ID) {   
    static constexpr uint8_t VOLTAGE_LEN = 1;
    float voltages[VOLTAGE_LEN] = {batteryData.pmData.busVoltage};

    // Pack battery data into telemetry message and send to TM
    TMMessage_t batteryDataMsg = batteryDataPack(
        systemUtilsDriver->getCurrentTimestampMs(),
        BATTERY_ID,
        INT16_MAX,
        voltages,
        VOLTAGE_LEN,
        batteryData.pmData.current,
        batteryData.pmData.charge,
        batteryData.pmData.energy,
        socEstimator.getSocPercentage(),
        socEstimator.getTimeRemaining(),
        batteryData.chargeState
    );
    tmQueue->push(&batteryDataMsg);
}

void SystemManager::sendStatusTextToTelemetryManager(MAV_SEVERITY severity, const char text[50], uint16_t id, uint8_t chunk_seq) {
    TMMessage_t statusTextMsg = statusTextPack(systemUtilsDriver->getCurrentTimestampMs(), severity, text, id, chunk_seq);
    tmQueue->push(&statusTextMsg);
}

PlaneFlightMode_e SystemManager::decodeRawFlightMode(float flightModeRawValue) {
    if (flightModeRawValue <= SM_FLIGHTMODE1_MAX) {
        return flightModes[0];
    }
    else if (flightModeRawValue <= SM_FLIGHTMODE2_MAX) {
        return flightModes[1];
    }
    else if (flightModeRawValue <= SM_FLIGHTMODE3_MAX) {
        return flightModes[2];
    }
    else if (flightModeRawValue <= SM_FLIGHTMODE4_MAX) {
        return flightModes[3];
    }
    else if (flightModeRawValue <= SM_FLIGHTMODE5_MAX) {
        return flightModes[4];
    } else {
        return flightModes[5];
    }
}

void SystemManager::sendMessagesToLogger() {
    static char messages[16][100];
    int msgCount = 0;

    while (smLoggerQueue->count() > 0) {
        smLoggerQueue->get(&messages[msgCount]);
        msgCount++;
    }

    // loggerDriver->log(messages, msgCount); (TODO: Uncomment after rearchitecture)
}
