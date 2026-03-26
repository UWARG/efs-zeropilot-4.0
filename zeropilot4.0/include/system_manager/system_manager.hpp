#pragma once

#include <cstdint>
#include "error.h"
#include "systemutils_iface.hpp"
#include "mavlink.h"
#include "logger_iface.hpp"
#include "rc_iface.hpp"
#include "rc_motor_control.hpp"
#include "iwdg_iface.hpp"
#include "tm_queue.hpp"
#include "queue_iface.hpp"
#include "power_module_iface.hpp"

#define SM_SCHEDULING_RATE_HZ 20
#define SM_TELEMETRY_HEARTBEAT_RATE_HZ 1
#define SM_TELEMETRY_RC_DATA_RATE_HZ 5
#define SM_TELEMETRY_BATTERY_DATA_RATE_HZ 1

#define SM_UPDATE_LOOP_DELAY_MS (1000 / SM_SCHEDULING_RATE_HZ)

// RC Arm threshold
static constexpr float SM_RC_ARM_THRESHOLD = 50.0f;

// Flightmode Count
static constexpr uint8_t SM_FLIGHTMODE_COUNT = 6;

// Calculated using 1165, 1295, 1425, 1555, 1685, and 1815 us as nominal values
static constexpr float SM_FLIGHTMODE1_MAX = 23.0f; // (1165 + 1295) / 2 = 1230 -> scaled/offset to 23.0
static constexpr float SM_FLIGHTMODE2_MAX = 36.0f; // (1295 + 1425) / 2 = 1360 -> scaled/offset to 36.0
static constexpr float SM_FLIGHTMODE3_MAX = 49.0f; // (1425 + 1555) / 2 = 1490 -> scaled/offset to 49.0
static constexpr float SM_FLIGHTMODE4_MAX = 62.0f; // (1555 + 1685) / 2 = 1620 -> scaled/offset to 62.0
static constexpr float SM_FLIGHTMODE5_MAX = 75.0f; // (1685 + 1815) / 2 = 1750 -> scaled/offset to 75.0

typedef struct{
    PMData_t pmData;
    MAV_BATTERY_CHARGE_STATE chargeState;
    uint32_t batteryLowCounterMs;
    uint32_t batteryCritcounterMs;
} BatteryData_t;

class SystemManager {
    public:
        SystemManager(
            ISystemUtils *systemUtilsDriver,
            IIndependentWatchdog *iwdgDriver,
            ILogger *loggerDriver,
            IRCReceiver *rcDriver,
            IPowerModule *pmDriver,
            IMessageQueue<RCMotorControlMessage_t> *amRCQueue,
            IMessageQueue<TMMessage_t> *tmQueue,
            IMessageQueue<char[100]> *smLoggerQueue
        );

        ZP_ERROR_e smUpdate(); // This function is the main function of SM, it should be called in the main loop of the system.

    private:
        ISystemUtils *systemUtilsDriver; // System utilities instance

        IIndependentWatchdog *iwdgDriver; // Independent Watchdog driver
        ILogger *loggerDriver; // Logger driver
        IRCReceiver *rcDriver; // RC receiver driver
        IPowerModule *pmDriver; // Power module driver
        
        IMessageQueue<RCMotorControlMessage_t> *amRCQueue; // Queue driver for tx communication to the Attitude Manager
        IMessageQueue<TMMessage_t> *tmQueue; // Queue driver for tx communication to the Telemetry Manager
        IMessageQueue<char[100]> *smLoggerQueue; // Queue driver for rx communication from other modules to the System Manager for logging

        uint8_t smSchedulingCounter;

        PlaneFlightMode_e flightModes[SM_FLIGHTMODE_COUNT];

        int oldDataCount;
        bool rcConnected;
        
        BatteryData_t batteryData;
        void updateBatteryFSM();

        ZP_ERROR_e sendRCDataToAttitudeManager(const RCControl &rcData);
        ZP_ERROR_e sendRCDataToTelemetryManager(const RCControl &rcData);
        ZP_ERROR_e sendHeartbeatDataToTelemetryManager(uint8_t baseMode, uint32_t customMode, MAV_STATE systemStatus);
        ZP_ERROR_e sendBatteryDataToTelemetryManager(const BatteryData_t &batteryData, const uint8_t BATTERY_ID);
        ZP_ERROR_e sendStatusTextToTelemetryManager(MAV_SEVERITY severity, const char text[50], uint16_t id = 0, uint8_t chunk_seq = 0);

        ZP_ERROR_e decodeRawFlightMode(PlaneFlightMode_e *decodedOutput, float flightModeRawValue);

        ZP_ERROR_e sendMessagesToLogger();

        // ZP_PARAM callbacks
        template <uint8_t Idx>
        static ZP_ERROR_e updateFltMode(bool *mode, SystemManager* context, float val);
};
