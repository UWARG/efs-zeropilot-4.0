#pragma once

#include "iwdg_iface.hpp"
#include "systemutils_iface.hpp"
#include "mavlink.h"
#include "logger_iface.hpp"
#include "rc_iface.hpp"
#include "rc_motor_control.hpp"
#include "iwdg_iface.hpp"
#include "m10_accessory_iface.hpp"
#include "tm_queue.hpp"
#include "queue_iface.hpp"
#include "power_module_iface.hpp"

#define SM_SCHEDULING_RATE_HZ 20
#define SM_TELEMETRY_HEARTBEAT_RATE_HZ 1
#define SM_TELEMETRY_RC_DATA_RATE_HZ 5

#define SM_UPDATE_LOOP_DELAY_MS (1000 / SM_SCHEDULING_RATE_HZ)
#define SM_RC_TIMEOUT_MS 500

#define SM_TELEMETRY_BATTERY_DATA_RATE_HZ 1
#define SM_BATTERY_LOW_TIME_MS 10000
#define SM_BATTERY_CRITICAL_TIME_MS 3000

// RC Arm threshold
static constexpr float SM_RC_ARM_THRESHOLD = 50.0f;

// Flightmode constants
static constexpr PlaneFlightMode_e SM_FLIGHTMODE1 = PlaneFlightMode_e::MANUAL;
static constexpr PlaneFlightMode_e SM_FLIGHTMODE2 = PlaneFlightMode_e::FBWA;
static constexpr PlaneFlightMode_e SM_FLIGHTMODE3 = PlaneFlightMode_e::MANUAL;
static constexpr PlaneFlightMode_e SM_FLIGHTMODE4 = PlaneFlightMode_e::MANUAL;
static constexpr PlaneFlightMode_e SM_FLIGHTMODE5 = PlaneFlightMode_e::MANUAL;
static constexpr PlaneFlightMode_e SM_FLIGHTMODE6 = PlaneFlightMode_e::MANUAL;

// Calculated using 1165, 1295, 1425, 1555, 1685, and 1815 us as nominal values
static constexpr float SM_FLIGHTMODE1_MAX = 23.0f; // (1165 + 1295) / 2 = 1230 -> scaled/offset to 23.0
static constexpr float SM_FLIGHTMODE2_MAX = 36.0f; // (1295 + 1425) / 2 = 1360 -> scaled/offset to 36.0
static constexpr float SM_FLIGHTMODE3_MAX = 49.0f; // (1425 + 1555) / 2 = 1490 -> scaled/offset to 49.0
static constexpr float SM_FLIGHTMODE4_MAX = 62.0f; // (1555 + 1685) / 2 = 1620 -> scaled/offset to 62.0
static constexpr float SM_FLIGHTMODE5_MAX = 75.0f; // (1685 + 1815) / 2 = 1750 -> scaled/offset to 75.0

// Battery related constants
static constexpr float BATTERY_LOW_VOLTAGE = 10.5f;
static constexpr float BATTERY_CRITICAL_VOLTAGE = 10.2f;
static constexpr float BATTERY_CAPACITY_MAH = 4000.0f;

static_assert(
    BATTERY_LOW_VOLTAGE > BATTERY_CRITICAL_VOLTAGE,
    "BATTERY_LOW_VOLTAGE must be greater than BATTERY_CRITICAL_VOLTAGE"
);

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
            IMessageQueue<char[100]> *smLoggerQueue,
            IM10Accessory *m10AccessoryDriver = nullptr
        );

        void smUpdate(); // This function is the main function of SM, it should be called in the main loop of the system.

    private:
        ISystemUtils *systemUtilsDriver; // System utilities instance

        IIndependentWatchdog *iwdgDriver; // Independent Watchdog driver
        ILogger *loggerDriver; // Logger driver
        IRCReceiver *rcDriver; // RC receiver driver
        IPowerModule *pmDriver; // Power module driver
        IM10Accessory *m10AccessoryDriver; // M10 safety switch / buzzer driver
        
        IMessageQueue<RCMotorControlMessage_t> *amRCQueue; // Queue driver for tx communication to the Attitude Manager
        IMessageQueue<TMMessage_t> *tmQueue; // Queue driver for tx communication to the Telemetry Manager
        IMessageQueue<char[100]> *smLoggerQueue; // Queue driver for rx communication from other modules to the System Manager for logging

        uint8_t smSchedulingCounter;

        int oldDataCount;
        bool rcConnected;
        
        BatteryData_t batteryData;
        void updateBatteryFSM();

        void sendRCDataToAttitudeManager(const RCControl &rcData, bool armed);
        void sendRCDataToTelemetryManager(const RCControl &rcData);
        void sendHeartbeatDataToTelemetryManager(uint8_t baseMode, uint32_t customMode, MAV_STATE systemStatus);
        void sendBatteryDataToTelemetryManager(const BatteryData_t &batteryData, const uint8_t BATTERY_ID);
        void sendStatusTextToTelemetryManager(MAV_SEVERITY severity, const char text[50], uint16_t id = 0, uint8_t chunk_seq = 0);

        PlaneFlightMode_e decodeRawFlightMode(float flightModeRawValue);

        void sendMessagesToLogger();
};
