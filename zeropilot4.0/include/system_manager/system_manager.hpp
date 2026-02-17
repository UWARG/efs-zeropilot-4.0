#pragma once

#include <vector>
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

#define SM_CONTROL_LOOP_DELAY 50
#define SM_RC_TIMEOUT 500

#define SM_SCHEDULING_RATE_HZ 20
#define SM_TELEMETRY_HEARTBEAT_RATE_HZ 1
#define SM_TELEMETRY_RC_DATA_RATE_HZ 5

#define SM_UPDATE_LOOP_DELAY_MS (1000 / SM_SCHEDULING_RATE_HZ)
#define SM_RC_TIMEOUT_MS 500

#define BATTERY_LOW_TIME_MS      10000
#define BATTERY_CRITICAL_TIME_MS 3000

static constexpr float BATTERY_LOW_VOLTAGE = 10.5f;
static constexpr float BATTERY_CRITICAL_VOLTAGE = 9.8f;

typedef struct{
    uint8_t batteryId;
    PMData_t pmData;
    MAV_BATTERY_CHARGE_STATE chargeState;
    uint32_t batteryLowCounterMs;
    uint32_t batteryCritcounterMs;
} BatteryData_t;

template<typename... Drivers>
struct AllDriversValid;

template<>
struct AllDriversValid<> : std::true_type {};

template<typename T, typename... Rest>
struct AllDriversValid<T, Rest...> 
    : std::conditional<
        std::is_base_of<IPowerModule, typename std::remove_pointer<T>::type>::value,
        AllDriversValid<Rest...>,
        std::false_type
    >::type {};

template<typename... Drivers>
constexpr bool pDriverTypeCheck() {
    return AllDriversValid<Drivers...>::value;
}

class SystemManager {
    public:
        template<typename... pmDriverType,
                typename = typename std::enable_if<pDriverTypeCheck<pmDriverType...>()>::type>
        SystemManager(
            ISystemUtils *systemUtilsDriver,
            IIndependentWatchdog *iwdgDriver,
            ILogger *loggerDriver,
            IRCReceiver *rcDriver,
            IMessageQueue<RCMotorControlMessage_t> *amRCQueue,
            IMessageQueue<TMMessage_t> *tmQueue,
            IMessageQueue<char[100]> *smLoggerQueue,
            pmDriverType*... pmDriver) :
                systemUtilsDriver(systemUtilsDriver),
                iwdgDriver(iwdgDriver),
                loggerDriver(loggerDriver),
                rcDriver(rcDriver),
                amRCQueue(amRCQueue),
                tmQueue(tmQueue),
                smLoggerQueue(smLoggerQueue),
                smSchedulingCounter(0),
                oldDataCount(0),
                rcConnected(false),
                batteryArray(sizeof...(pmDriver)),
                pmDrivers{pmDriver...}{
                    for (size_t i = 0; i < batteryArray.size(); i++){
                        batteryArray[i].batteryId = i;
                        batteryArray[i].chargeState = MAV_BATTERY_CHARGE_STATE_UNDEFINED;
                        batteryArray[i].batteryLowCounterMs = 0;
                        batteryArray[i].batteryCritcounterMs = 0;
                    }
            }

        void smUpdate(); // This function is the main function of SM, it should be called in the main loop of the system.

    private:
        ISystemUtils *systemUtilsDriver; // System utilities instance

        IIndependentWatchdog *iwdgDriver; // Independent Watchdog driver
        ILogger *loggerDriver; // Logger driver
        IRCReceiver *rcDriver; // RC receiver driver    
        std::vector<IPowerModule*> pmDrivers;
        
        IMessageQueue<RCMotorControlMessage_t> *amRCQueue; // Queue driver for tx communication to the Attitude Manager
        IMessageQueue<TMMessage_t> *tmQueue; // Queue driver for tx communication to the Telemetry Manager
        IMessageQueue<char[100]> *smLoggerQueue; // Queue driver for rx communication from other modules to the System Manager for logging

        uint8_t smSchedulingCounter;

        int oldDataCount;
        bool rcConnected;
        
        std::vector<BatteryData_t> batteryArray;

        void sendRCDataToAttitudeManager(const RCControl &rcData);
        void sendRCDataToTelemetryManager(const RCControl &rcData);
        void sendHeartbeatDataToTelemetryManager(uint8_t baseMode, uint32_t customMode, MAV_STATE systemStatus);
        void sendBMDataToTelemetryManager(const BatteryData_t &batteryData);
        void sendMessagesToLogger();
};
