#pragma once

#include "iwdg_iface.hpp"
#include "systemutils_iface.hpp"
#include "mavlink.h"
#include "logger_iface.hpp"
#include "rc_iface.hpp"
#include "rc_motor_control.hpp"
#include "iwdg_iface.hpp"
#include "safety_switch_iface.hpp"
#include "tm_queue.hpp"
#include "queue_iface.hpp"
#include "power_module_iface.hpp"
#include "sm_param_setup.hpp"
#include "zp_error.h"
#include "zp_bit.hpp"
#include "soc_estimation.hpp"

#define SM_SCHEDULING_RATE_HZ 20
#define SM_TELEMETRY_HEARTBEAT_RATE_HZ 1
#define SM_TELEMETRY_RC_DATA_RATE_HZ 5
#define SM_TELEMETRY_BATTERY_DATA_RATE_HZ 1
#define SM_TELEMETRY_SYS_STATUS_RATE_HZ 1
#define SM_TELEMETRY_BIT_FAIL_PERIOD_S 5

#define SM_UPDATE_LOOP_DELAY_MS (1000 / SM_SCHEDULING_RATE_HZ)

// RC Arm threshold
static constexpr float SM_RC_ARM_THRESHOLD = 50.0f;

// Flightmode Count
static constexpr uint8_t SM_FLIGHTMODE_COUNT = 6;
static constexpr uint8_t SM_RC_REVERSIBLE_COUNT = 4;

// Calculated using 1165, 1295, 1425, 1555, 1685, and 1815 us as nominal values
static constexpr float SM_FLIGHTMODE1_MAX = 23.0f; // (1165 + 1295) / 2 = 1230 -> scaled/offset to 23.0
static constexpr float SM_FLIGHTMODE2_MAX = 36.0f; // (1295 + 1425) / 2 = 1360 -> scaled/offset to 36.0
static constexpr float SM_FLIGHTMODE3_MAX = 49.0f; // (1425 + 1555) / 2 = 1490 -> scaled/offset to 49.0
static constexpr float SM_FLIGHTMODE4_MAX = 62.0f; // (1555 + 1685) / 2 = 1620 -> scaled/offset to 62.0
static constexpr float SM_FLIGHTMODE5_MAX = 75.0f; // (1685 + 1815) / 2 = 1750 -> scaled/offset to 75.0

// Safety switch constants
static constexpr uint32_t SM_SAFETY_SWITCH_HOLD_THRESHOLD_MS = 2000;
static constexpr uint32_t SM_SAFETY_SWITCH_BLINK_RATE_HZ = 2;
static constexpr uint32_t SM_SAFETY_SWITCH_PREARM_MSG_INTERVAL_S = 10; // Send safety switch prearm message every 10 seconds

class SystemManager {
    friend class SMParamSetup;

    public:
        SystemManager(
            ISystemUtils *systemUtilsDriver,
            IIndependentWatchdog *iwdgDriver,
            ILogger *loggerDriver,
            ISafetySwitch *safetySwitchDriver,
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
        ISafetySwitch *safetySwitchDriver; // Safety switch driver
        IRCReceiver *rcDriver; // RC receiver driver
        IPowerModule *pmDriver; // Power module driver
        
        IMessageQueue<RCMotorControlMessage_t> *amRCQueue; // Queue driver for tx communication to the Attitude Manager
        IMessageQueue<TMMessage_t> *tmQueue; // Queue driver for tx communication to the Telemetry Manager
        IMessageQueue<char[100]> *smLoggerQueue; // Queue driver for rx communication from other modules to the System Manager for logging

        uint8_t smSchedulingCounter;

        FlightMode_e flightModes[SM_FLIGHTMODE_COUNT];

        bool isSafetySwitchEngaged;         // Flag to indicate if the safety switch is engaged
        uint32_t safetySwitchHoldCounterMs; // Counter to track how long the safety switch has been held
        bool safetySwitchTriggered;         // Flag to prevent toggling multiple times during a single long press
        uint32_t safetySwitchPrearmCntrMs;  // Counter to track time since last prearm message was sent
        
        // Ordered by severity so arbitration can pick the highest
        enum class BitFailsafe_e : uint8_t {
            NONE,   // No action
            REPORT, // Report the error
            DISARM  // Disarm
        };

        typedef struct {
            ZP_BIT_ID id;
            uint32_t mavSensorBit; // MAV_SYS_STATUS_SENSOR_*, 0 if unmapped
            const char* failText;
            BitFailsafe_e failsafe;
        } BitHandler_t;

        static const BitHandler_t BIT_HANDLERS[static_cast<uint16_t>(ZP_BIT_ID::NUM_BIT_IDS)];

        ZP_Error initBitHandlers();

        // Sends failure text for every failing BIT and arbitrates bitFailsafe from their actions
        ZP_Error applyBitFailsafes();
        ZP_Error reportLoopTiming(ZP_BIT_ID id, uint32_t maxExecUs, uint32_t budgetMs);

        ZP_Error safetySwitchUpdate();

        bool rcConnected;
        BitFailsafe_e bitFailsafe; // most severe action requested by a failing BIT
        uint32_t bitFailReportCntrMs; // time since the failing BITs were last reported

        bool rcChannelReversed[SM_RC_REVERSIBLE_COUNT];
        
        BatteryData_t batteryData;
        ZP_Error updateBatteryFSM();
        SocEstimator socEstimator;

        ZP_Error sendRCDataToAttitudeManager(const RCControl &rcData);
        ZP_Error sendRCDataToTelemetryManager(const RCControl &rcData);
        ZP_Error sendHeartbeatDataToTelemetryManager(uint8_t baseMode, uint32_t customMode, MAV_STATE systemStatus);
        ZP_Error sendSysStatusToTelemetryManager();

        // ZP_ERROR_OK means armable. Otherwise outFirstBlocking is the first BIT that blocks arming
        ZP_Error prearmCheck(ZP_BIT_ID& outFirstBlocking);

        ZP_Error getHealthMask(uint32_t& outPresent, uint32_t& outEnabled, uint32_t& outHealth);
        ZP_Error sendBatteryDataToTelemetryManager(const BatteryData_t &batteryData, const uint8_t batteryId);
        ZP_Error sendStatusTextToTelemetryManager(MAV_SEVERITY severity, const char text[50], uint16_t id = 0, uint8_t chunk_seq = 0);

        ZP_Error decodeRawFlightMode(float flightModeRawValue, FlightMode_e& flightMode);

        ZP_Error sendMessagesToLogger();

        uint8_t profilerId;

        SMParamSetup paramSetup;

        uint8_t profilerBuf[256];
        TaskProfile profiles[MAX_PROFILED_TASKS];
};

inline const SystemManager::BitHandler_t SystemManager::BIT_HANDLERS[static_cast<uint16_t>(ZP_BIT_ID::NUM_BIT_IDS)] = {
    {ZP_BIT_ID::PARAM_TABLE_INIT,       0,                                       "PreArm: Param table init failed", BitFailsafe_e::REPORT},
    {ZP_BIT_ID::IMU_INIT,               MAV_SYS_STATUS_SENSOR_3D_GYRO,           "PreArm: IMU init failed",         BitFailsafe_e::REPORT},
    {ZP_BIT_ID::GPS1_INIT,              MAV_SYS_STATUS_SENSOR_GPS,               "GPS1 init failed",                BitFailsafe_e::REPORT},
    {ZP_BIT_ID::GPS2_INIT,              MAV_SYS_STATUS_SENSOR_GPS,               "GPS2 init failed",                BitFailsafe_e::REPORT},
    {ZP_BIT_ID::BARO_INIT,              MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE, "Baro init failed",                BitFailsafe_e::REPORT},
    {ZP_BIT_ID::RC_INIT,                MAV_SYS_STATUS_SENSOR_RC_RECEIVER,       "PreArm: RC init failed",          BitFailsafe_e::REPORT},
    {ZP_BIT_ID::PM_INIT,                MAV_SYS_STATUS_SENSOR_BATTERY,           "Power module init failed",        BitFailsafe_e::REPORT},
    {ZP_BIT_ID::TELEM_INIT,             0,                                       "Telemetry init failed",           BitFailsafe_e::REPORT},
    {ZP_BIT_ID::RANGEFINDER_INIT,       MAV_SYS_STATUS_SENSOR_LASER_POSITION,    "Rangefinder init failed",         BitFailsafe_e::REPORT},
    {ZP_BIT_ID::MOTOR_INIT,             MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS,     "PreArm: Motor init failed",       BitFailsafe_e::REPORT},
    {ZP_BIT_ID::CAN_INIT,               0,                                       "CAN init failed",                 BitFailsafe_e::REPORT},
    {ZP_BIT_ID::RC_DATA_VALID,          MAV_SYS_STATUS_SENSOR_RC_RECEIVER,       "PreArm: RC disconnected",         BitFailsafe_e::DISARM},
    {ZP_BIT_ID::IMU_DATA_VALID,         MAV_SYS_STATUS_SENSOR_3D_GYRO,           "PreArm: IMU data invalid",        BitFailsafe_e::REPORT},
    {ZP_BIT_ID::GPS_DATA_VALID,         MAV_SYS_STATUS_SENSOR_GPS,               "GPS data invalid",                BitFailsafe_e::REPORT},
    {ZP_BIT_ID::BARO_DATA_VALID,        MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE, "Baro data invalid",               BitFailsafe_e::REPORT},
    {ZP_BIT_ID::PM_DATA_VALID,          MAV_SYS_STATUS_SENSOR_BATTERY,           "Power module data invalid",       BitFailsafe_e::REPORT},
    {ZP_BIT_ID::RANGEFINDER_DATA_VALID, MAV_SYS_STATUS_SENSOR_LASER_POSITION,    "Rangefinder data invalid",        BitFailsafe_e::REPORT},
    {ZP_BIT_ID::TELEM_LINK_VALID,       0,                                       "Telemetry link lost",             BitFailsafe_e::REPORT},
    {ZP_BIT_ID::BATT_LOW,               MAV_SYS_STATUS_SENSOR_BATTERY,           "Battery low",                     BitFailsafe_e::REPORT},
    {ZP_BIT_ID::BATT_CRITICAL,          MAV_SYS_STATUS_SENSOR_BATTERY,           "PreArm: Battery critical",        BitFailsafe_e::REPORT},
    {ZP_BIT_ID::AM_LOOP_TIMING,         0,                                       "AM loop overrun",                 BitFailsafe_e::REPORT},
    {ZP_BIT_ID::SM_LOOP_TIMING,         0,                                       "SM loop overrun",                 BitFailsafe_e::REPORT},
    {ZP_BIT_ID::TM_LOOP_TIMING,         0,                                       "TM loop overrun",                 BitFailsafe_e::REPORT},
    {ZP_BIT_ID::IWDG_REFRESH,           0,                                       "PreArm: Watchdog refresh failed", BitFailsafe_e::REPORT},
    {ZP_BIT_ID::LOGGER_VALID,           0,                                       "Logger failed",                   BitFailsafe_e::REPORT},
};
