#pragma once

#include "iwdg_iface.hpp"
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

#define BATTERY_NCELLS 3
#define SOC_IDLE_MODE 0
#define SOC_CHARGE_DISCHARGE_MODE 1

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

static constexpr voltageToSoc_t socLUT[] = {
    {4.20f, 100.0f},
    {4.15f, 95.0f},
    {4.11f, 90.0f},
    {4.08f, 85.0f},
    {4.02f, 80.0f},
    {3.98f, 75.0f},
    {3.95f, 70.0f},
    {3.91f, 65.0f},
    {3.87f, 60.0f},
    {3.85f, 55.0f},
    {3.84f, 50.0f},
    {3.82f, 45.0f},
    {3.80f, 40.0f},
    {3.79f, 35.0f},
    {3.77f, 30.0f},
    {3.75f, 25.0f},
    {3.73f, 20.0f},
    {3.71f, 15.0f},
    {3.69f, 10.0f},
    {3.61f, 5.0f},
    {3.50f, 0.0f}
};

typedef struct {
    float voltage;
    float soc;
} voltageToSoc_t;

typedef struct {
    uint8_t socPercentage;
    int32_t timeRemaining;
} StateOfCharge_t;

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

        void smUpdate(); // This function is the main function of SM, it should be called in the main loop of the system.

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

        StateOfCharge_t socData;
        void calcStateOfCharge(int mode);

        void sendRCDataToAttitudeManager(const RCControl &rcData);
        void sendRCDataToTelemetryManager(const RCControl &rcData);
        void sendHeartbeatDataToTelemetryManager(uint8_t baseMode, uint32_t customMode, MAV_STATE systemStatus);
        void sendBatteryDataToTelemetryManager(const BatteryData_t &batteryData, const uint8_t BATTERY_ID);
        void sendStatusTextToTelemetryManager(MAV_SEVERITY severity, const char text[50], uint16_t id = 0, uint8_t chunk_seq = 0);

        PlaneFlightMode_e decodeRawFlightMode(float flightModeRawValue);

        void sendMessagesToLogger();

        // ZP_PARAM callbacks
        template <uint8_t Idx>
        static bool updateFltMode(SystemManager* context, float val);
};
