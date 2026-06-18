#include "system_manager.hpp"
#include "zp_params.hpp"
#include "flightmode.hpp"
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
        profilerId(0),
        paramSetup(this)
{
    paramSetup.loadAllParams();
    paramSetup.bindAllParamCallbacks();
    systemUtilsDriver->profilerRegister("SM", &profilerId);
}

ZP_ERROR_e SystemManager::smUpdate() {
    ZP_ERROR_e result = ZP_ERROR_OK;
    systemUtilsDriver->profilerBegin(profilerId);

    // Kick the watchdog
    result |= iwdgDriver->refreshWatchdog();

    // Get RC data from the RC receiver and passthrough to AM if new
    RCControl rcData;
    result |= rcDriver->getRCData(rcData);
    
    if (result == ZP_ERROR_OK) {
        if (rcData.isDataNew) {
            oldDataCount = 0;
            result |= sendRCDataToAttitudeManager(rcData);

            if (!rcConnected) {
                result |= sendStatusTextToTelemetryManager(MAV_SEVERITY_INFO, "RC Connected");
                // loggerDriver->log("RC Connected"); (TODO: Uncomment after rearchitecture)
                rcConnected = true;
            }
        } else {
            oldDataCount += 1;

            float fsTimeout = 0.0f;
            if (ZP_PARAM::get(ZP_PARAM_ID::RC_FS_TIMEOUT, fsTimeout) == ZP_ERROR_OK) {
                if ((oldDataCount * SM_UPDATE_LOOP_DELAY_MS > (fsTimeout * 1000)) && rcConnected) {
                    result |= sendStatusTextToTelemetryManager(MAV_SEVERITY_CRITICAL, "RC Disconnected");
                    // loggerDriver->log("RC Disconnected"); (TODO: Uncomment after rearchitecture)
                    rcConnected = false;
                }
            }
        }
    }

    // Send RC data to TM
    if (smSchedulingCounter % (SM_SCHEDULING_RATE_HZ / SM_TELEMETRY_RC_DATA_RATE_HZ) == 0) {
        result |= sendRCDataToTelemetryManager(rcData);
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
    PlaneFlightMode_e flightMode;
    result |= decodeRawFlightMode(rcData.fltModeRaw, flightMode);
    uint32_t customMode = static_cast<uint32_t>(flightMode);

    // Send Heartbeat data to TM at a 1Hz rate
    if (smSchedulingCounter % (SM_SCHEDULING_RATE_HZ / SM_TELEMETRY_HEARTBEAT_RATE_HZ) == 0) {
        result |= sendHeartbeatDataToTelemetryManager(baseMode, customMode, systemStatus);
    }

    // Monitor Battery State and send Battery Data to TM at a 1Hz rate
    if (updateBatteryFSM() == ZP_ERROR_OK) {
        if (smSchedulingCounter % (SM_SCHEDULING_RATE_HZ / SM_TELEMETRY_BATTERY_DATA_RATE_HZ) == 0) {
            result |= sendBatteryDataToTelemetryManager(batteryData, 0);
        }
    }

    // Log if new messages
    int counter = 0;
    result |= smLoggerQueue->count(counter);
    if (counter > 0 && result == ZP_ERROR_OK) {
        result |= sendMessagesToLogger();
    }

    // Send profiler stats at 1Hz
    if (smSchedulingCounter % (SM_SCHEDULING_RATE_HZ / SM_TELEMETRY_HEARTBEAT_RATE_HZ) == 0) {
        uint8_t count = 0;
        systemUtilsDriver->profilerGetAll(profiles, &count);

        for (uint8_t i = 0; i < count; i++) {
            if (strcmp(profiles[i].name, "SM") == 0) {
                if (profiles[i].deltaExec >= (SM_UPDATE_LOOP_DELAY_MS * 1000)) {
                    result |= sendStatusTextToTelemetryManager(MAV_SEVERITY_CRITICAL, "SM execution time exceeding scheduled rate");
                } else if (profiles[i].deltaExec >= 0.8f * (SM_UPDATE_LOOP_DELAY_MS * 1000)) {
                    result |= sendStatusTextToTelemetryManager(MAV_SEVERITY_WARNING, "SM execution time about to exceed scheduled rate");
                }
            } else if (strcmp(profiles[i].name, "AM") == 0) {
                if (profiles[i].deltaExec >= (AM_UPDATE_LOOP_DELAY_MS * 1000)) {
                    result |= sendStatusTextToTelemetryManager(MAV_SEVERITY_CRITICAL, "AM execution time exceeding scheduled rate");
                } else if (profiles[i].deltaExec >= 0.8f * (AM_UPDATE_LOOP_DELAY_MS * 1000)) {
                    result |= sendStatusTextToTelemetryManager(MAV_SEVERITY_WARNING, "AM execution time about to exceed scheduled rate");
                }
            } else if (strcmp(profiles[i].name, "TM") == 0) {
                if (profiles[i].deltaExec >= (TM_UPDATE_LOOP_DELAY_MS * 1000)) {
                    result |= sendStatusTextToTelemetryManager(MAV_SEVERITY_CRITICAL, "TM execution time exceeding scheduled rate");
                } else if (profiles[i].deltaExec >= 0.8f * (TM_UPDATE_LOOP_DELAY_MS * 1000)) {
                    result |= sendStatusTextToTelemetryManager(MAV_SEVERITY_WARNING, "TM execution time about to exceed scheduled rate");
                }
            }
            #if LOG_TIMING
            snprintf((char*)profilerBuf, sizeof(profilerBuf), "%-12s %lu us      %lu hz", profiles[i].name, profiles[i].deltaExec, profiles[i].deltaPeriod);
            result |= sendStatusTextToTelemetryManager(MAV_SEVERITY_INFO, (char*)profilerBuf);
            #endif
        }
        #if LOG_TIMING
        result |= sendStatusTextToTelemetryManager(MAV_SEVERITY_INFO, "-------TASK TIMINGS-------");
        #endif
    }

    // Increment scheduling counter
    smSchedulingCounter = (smSchedulingCounter + 1) % SM_SCHEDULING_RATE_HZ;

    systemUtilsDriver->profilerEnd(profilerId);
    return result;
}

ZP_ERROR_e SystemManager::updateBatteryFSM() {
    ZP_ERROR_e result = ZP_ERROR_OK;
    batteryData.isValid = false;         
    MAV_BATTERY_CHARGE_STATE currentBatteryState;

    result |= pmDriver->readData(&batteryData.pmData);

    if (result == ZP_ERROR_OK) {
        batteryData.isValid = true;         
        currentBatteryState = batteryData.chargeState;

        float lowVolt = 0.0f;
        float critVolt = 0.0f;
        float lowTimer = 0.0f;
        
        // Accumulate errors from parameter lookups
        result |= ZP_PARAM::get(ZP_PARAM_ID::BATT_LOW_VOLT, lowVolt);
        result |= ZP_PARAM::get(ZP_PARAM_ID::BATT_CRT_VOLT, critVolt);
        result |= ZP_PARAM::get(ZP_PARAM_ID::BATT_LOW_TIMER, lowTimer);
        uint32_t battLowTimeMs = static_cast<uint32_t>(lowTimer * 1000.0f);
        if (result == ZP_ERROR_OK) {
            if (batteryData.pmData.busVoltage >= lowVolt) {
                // Normal battery
                batteryData.chargeState = MAV_BATTERY_CHARGE_STATE_OK;
                batteryData.batteryLowCounterMs = 0;
                batteryData.batteryCritcounterMs = 0;
            } else if (batteryData.pmData.busVoltage >= critVolt) {
                // Low battery detection
                batteryData.batteryLowCounterMs += SM_UPDATE_LOOP_DELAY_MS;
                batteryData.batteryCritcounterMs = 0;
                if (battLowTimeMs > 0 && batteryData.batteryLowCounterMs >= battLowTimeMs) {
                    batteryData.chargeState = MAV_BATTERY_CHARGE_STATE_LOW;
                }
            } else {
                // Critical battery detection
                batteryData.batteryCritcounterMs += SM_UPDATE_LOOP_DELAY_MS;
                batteryData.batteryLowCounterMs = 0;
                if (battLowTimeMs > 0 && batteryData.batteryCritcounterMs >= battLowTimeMs) {
                    batteryData.chargeState = MAV_BATTERY_CHARGE_STATE_CRITICAL;
                }
            }

            // Logging --> once per transition, checks if the state has yet to be logged and does so 
            if (currentBatteryState != batteryData.chargeState) {
                switch (batteryData.chargeState) {
                    case MAV_BATTERY_CHARGE_STATE_OK:
                        result |= sendStatusTextToTelemetryManager(MAV_SEVERITY_INFO, "Battery State: OK");
                        break;
                    case MAV_BATTERY_CHARGE_STATE_LOW:
                        result |= sendStatusTextToTelemetryManager(MAV_SEVERITY_WARNING, "Battery State: LOW");
                        break;
                    case MAV_BATTERY_CHARGE_STATE_CRITICAL:
                        result |= sendStatusTextToTelemetryManager(MAV_SEVERITY_CRITICAL, "Battery State: CRITICAL");
                        break;
                    default:
                        break;
                }
            }
        }
    }

    return result;
}

ZP_ERROR_e SystemManager::sendRCDataToTelemetryManager(const RCControl &rcData) {
    ZP_ERROR_e result = ZP_ERROR_OK;
    TMMessage_t rcDataMsg;
    uint32_t currentTime = 0;
    result |= systemUtilsDriver->getCurrentTimestampMs(currentTime);
    result |= rcDataPack(rcDataMsg, currentTime, rcData.controlSignals, INPUT_CHANNELS);
    
    if (result == ZP_ERROR_OK) {
        result |= tmQueue->push(&rcDataMsg);
    }
    return result;
}

ZP_ERROR_e SystemManager::sendHeartbeatDataToTelemetryManager(uint8_t baseMode, uint32_t customMode, MAV_STATE systemStatus) {
    ZP_ERROR_e result = ZP_ERROR_OK;
    TMMessage_t hbDataMsg;
    uint32_t currentTime = 0;
    result |= systemUtilsDriver->getCurrentTimestampMs(currentTime);
    result |= heartbeatPack(hbDataMsg, currentTime, baseMode, customMode, systemStatus);
    
    if (result == ZP_ERROR_OK) {
        result |= tmQueue->push(&hbDataMsg);
    }
    return result;
}

ZP_ERROR_e SystemManager::sendRCDataToAttitudeManager(const RCControl &rcData) {
    RCMotorControlMessage_t rcDataMessage;
    PlaneFlightMode_e fltMode;

    ZP_ERROR_e result = decodeRawFlightMode(rcData.fltModeRaw, fltMode);

    if (result == ZP_ERROR_OK) {
        rcDataMessage.roll = rcData.roll;
        rcDataMessage.pitch = rcData.pitch;
        rcDataMessage.yaw = rcData.yaw;
        rcDataMessage.throttle = rcData.throttle;
        rcDataMessage.arm = rcData.arm > SM_RC_ARM_THRESHOLD;
        rcDataMessage.flapAngle = rcData.aux2;
        rcDataMessage.flightMode = fltMode;

        result |= amRCQueue->push(&rcDataMessage);
    }
    return result;
}

ZP_ERROR_e SystemManager::sendBatteryDataToTelemetryManager(const BatteryData_t &batteryData, const uint8_t BATTERY_ID) {   
    static constexpr uint8_t VOLTAGE_LEN = 1;
    float voltages[VOLTAGE_LEN] = {batteryData.pmData.busVoltage};

    float battCapacityMah = 0.0f;
    ZP_ERROR_e result = ZP_PARAM::get(ZP_PARAM_ID::BATT_CAPACITY, battCapacityMah);

    if (result == ZP_ERROR_OK) {
        float consumedColoumbs = batteryData.pmData.charge;
        float remainingColoumbs = (battCapacityMah * 3.6f) - consumedColoumbs;
        remainingColoumbs = remainingColoumbs < 0 ? 0 : remainingColoumbs;
        int8_t socPercentage = static_cast<int8_t>((remainingColoumbs / (battCapacityMah * 3.6f)) * 100.0f);

        int32_t timeRemainingSec = 0;
        if (batteryData.pmData.current > 0.5f) {
            timeRemainingSec = static_cast<int32_t>(remainingColoumbs / batteryData.pmData.current);
        }

        TMMessage_t batteryDataMsg;
        uint32_t currentTime = 0;
        result |= systemUtilsDriver->getCurrentTimestampMs(currentTime);
        result |= batteryDataPack(batteryDataMsg, currentTime, BATTERY_ID, 
                                  INT16_MAX, voltages, VOLTAGE_LEN, batteryData.pmData.current, 
                                  static_cast<int32_t>(batteryData.pmData.charge), 
                                  static_cast<int32_t>(batteryData.pmData.energy), 
                                  socPercentage, timeRemainingSec, batteryData.chargeState);
        
        if (result == ZP_ERROR_OK) {
            result |= tmQueue->push(&batteryDataMsg);
        }
    }
    return result;
}

ZP_ERROR_e SystemManager::sendStatusTextToTelemetryManager(MAV_SEVERITY severity, const char text[50], uint16_t id, uint8_t chunk_seq) {
    ZP_ERROR_e result = ZP_ERROR_OK;
    TMMessage_t statusTextMsg;
    uint32_t currentTime = 0;
    result |= systemUtilsDriver->getCurrentTimestampMs(currentTime);
    result |= statusTextPack(statusTextMsg, currentTime, severity, text, id, chunk_seq);
    
    if (result == ZP_ERROR_OK) {
        result |= tmQueue->push(&statusTextMsg);
    }
    return result;
}

ZP_ERROR_e SystemManager::decodeRawFlightMode(float flightModeRawValue, PlaneFlightMode_e &outMode) {
    if (flightModeRawValue <= SM_FLIGHTMODE1_MAX) outMode = flightModes[0];
    else if (flightModeRawValue <= SM_FLIGHTMODE2_MAX) outMode = flightModes[1];
    else if (flightModeRawValue <= SM_FLIGHTMODE3_MAX) outMode = flightModes[2];
    else if (flightModeRawValue <= SM_FLIGHTMODE4_MAX) outMode = flightModes[3];
    else if (flightModeRawValue <= SM_FLIGHTMODE5_MAX) outMode = flightModes[4];
    else outMode = flightModes[5];
    
    return ZP_ERROR_OK;
}

ZP_ERROR_e SystemManager::sendMessagesToLogger() {
    ZP_ERROR_e result = ZP_ERROR_OK;
    static char messages[16][100];
    int msgCount = 0;
    int queueCount = 0;

    result |= smLoggerQueue->count(queueCount);

    if (result == ZP_ERROR_OK) {
        while (queueCount-- > 0) {
            result |= smLoggerQueue->get(&messages[msgCount]);
            if (result != ZP_ERROR_OK) break;
            msgCount++;
        }
    }
    return result;
}