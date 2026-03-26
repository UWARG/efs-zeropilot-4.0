#include "system_manager.hpp"
#include "zp_params.hpp"
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
        flightModes{},
        oldDataCount(0),
        rcConnected(false),
        batteryData({PMData_t{}, MAV_BATTERY_CHARGE_STATE_OK, 0, 0})
{
    static constexpr ZP_PARAM_ID FLTMODE_PARAMS[SM_FLIGHTMODE_COUNT] = {
        ZP_PARAM_ID::FLTMODE1, ZP_PARAM_ID::FLTMODE2, ZP_PARAM_ID::FLTMODE3,
        ZP_PARAM_ID::FLTMODE4, ZP_PARAM_ID::FLTMODE5, ZP_PARAM_ID::FLTMODE6
    };
    for (uint8_t i = 0; i < SM_FLIGHTMODE_COUNT; i++) {
        flightModes[i] = static_cast<PlaneFlightMode_e>(static_cast<uint32_t>(ZP_PARAM::get(FLTMODE_PARAMS[i])));
    }

    ZP_PARAM::bindCallback(ZP_PARAM_ID::FLTMODE1, this, SystemManager::updateFltMode<0>);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::FLTMODE2, this, SystemManager::updateFltMode<1>);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::FLTMODE3, this, SystemManager::updateFltMode<2>);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::FLTMODE4, this, SystemManager::updateFltMode<3>);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::FLTMODE5, this, SystemManager::updateFltMode<4>);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::FLTMODE6, this, SystemManager::updateFltMode<5>);
}

void SystemManager::smUpdate() {
    // Kick the watchdog
    ZP_RETURN_IF_ERROR(iwdgDriver->refreshWatchdog());


    // Get RC data from the RC receiver and passthrough to AM if new
    RCControl rcData;
    ZP_RETURN_IF_ERROR(rcDriver->getRCData(&rcData));

    if (rcData.isDataNew) {
        oldDataCount = 0;
        ZP_RETURN_IF_ERROR(sendRCDataToAttitudeManager(rcData));

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
        ZP_RETURN_IF_ERROR(sendRCDataToTelemetryManager(rcData));
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
        ZP_RETURN_IF_ERROR(sendHeartbeatDataToTelemetryManager(baseMode, customMode, systemStatus));
    }

    // Monitor Battery State and send Battery Data to TM at a 1Hz rate
    ZP_RETURN_IF_ERROR(updateBatteryFSM());
    if (smSchedulingCounter % (SM_SCHEDULING_RATE_HZ / SM_TELEMETRY_BATTERY_DATA_RATE_HZ) == 0) {
        ZP_RETURN_IF_ERROR(sendBatteryDataToTelemetryManager(batteryData, 0));
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

ZP_ERROR_e SystemManager::updateBatteryFSM() {
    MAV_BATTERY_CHARGE_STATE currentBatteryState;
    if (pmDriver->readData(&batteryData.pmData)) {          
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
    }

    return ZP_ERROR_OK;
}

ZP_ERROR_e SystemManager::sendRCDataToTelemetryManager(const RCControl &rcData) {
    TMMessage_t rcDataMsg = rcDataPack(systemUtilsDriver->getCurrentTimestampMs(), rcData.controlSignals, INPUT_CHANNELS);
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
    rcDataMessage.arm = rcData.arm > SM_RC_ARM_THRESHOLD;
    rcDataMessage.flapAngle = rcData.aux2;
    rcDataMessage.flightMode = decodeRawFlightMode(rcData.fltModeRaw);

    amRCQueue->push(&rcDataMessage);
}

ZP_ERROR_e SystemManager::sendBatteryDataToTelemetryManager(const BatteryData_t &batteryData, const uint8_t BATTERY_ID) {   
    static constexpr uint8_t VOLTAGE_LEN = 1;
    float voltages[VOLTAGE_LEN] = {batteryData.pmData.busVoltage};

    // Get battery capacity from ZP_PARAM
    float battCapacityMah = ZP_PARAM::get(ZP_PARAM_ID::BATT_CAPACITY);

    // SOC estimation (0-100 %) based on capacity
    float consumedColoumbs = batteryData.pmData.charge;
    float remainingColoumbs = (battCapacityMah * 3.6f) - consumedColoumbs;
    remainingColoumbs = remainingColoumbs < 0 ? 0 : remainingColoumbs; // Floor at 0
    int8_t socPercentage = static_cast<int8_t>((remainingColoumbs / (battCapacityMah * 3.6f)) * 100.0f);

    // Simple time remaining estimation based on current consumption
    int32_t timeRemainingSec = 0; // Default to unknown if current is too low to estimate
    if (batteryData.pmData.current > 0.5f) {
        timeRemainingSec = static_cast<int32_t>(remainingColoumbs / batteryData.pmData.current);
    }

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
        socPercentage,
        timeRemainingSec,
        batteryData.chargeState
    );
    ZP_RETURN_IF_ERROR(tmQueue->push(&batteryDataMsg));
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
    int queueCount = 0;

    ZP_RETURN_IF_ERROR(smLoggerQueue->count(&queueCount));

    while (queueCount > 0) {
        ZP_RETURN_IF_ERROR(smLoggerQueue->get(&messages[msgCount]));
        msgCount++;

        ZP_RETURN_IF_ERROR(smLoggerQueue->count(&queueCount));
    }

    // loggerDriver->log(messages, msgCount); (TODO: Uncomment after rearchitecture)
}

// Flight mode param callback (template instantiated per slot)
template <uint8_t Idx>
bool SystemManager::updateFltMode(SystemManager* context, float val) {
    uint32_t mode = static_cast<uint32_t>(val);
    if (!isValidPlaneFlightMode(mode)) return false;
    context->flightModes[Idx] = static_cast<PlaneFlightMode_e>(mode);
    return true;
}

// STATIC FUNCTIONS ONLY FOR PARAM CHAINING
// ==============================================================
template bool SystemManager::updateFltMode<0>(SystemManager*, float);
template bool SystemManager::updateFltMode<1>(SystemManager*, float);
template bool SystemManager::updateFltMode<2>(SystemManager*, float);
template bool SystemManager::updateFltMode<3>(SystemManager*, float);
template bool SystemManager::updateFltMode<4>(SystemManager*, float);
template bool SystemManager::updateFltMode<5>(SystemManager*, float);
// ==============================================================
