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

#define SM_CONTROL_LOOP_DELAY 50
#define SM_RC_TIMEOUT 500 

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
        IPowerModule *pmDriver;
        
        IMessageQueue<RCMotorControlMessage_t> *amRCQueue; // Queue driver for tx communication to the Attitude Manager
        IMessageQueue<TMMessage_t> *tmQueue; // Queue driver for tx communication to the Telemetry Manager
        IMessageQueue<char[100]> *smLoggerQueue; // Queue driver for rx communication from other modules to the System Manager for logging

        uint8_t smSchedulingCounter;

        void sendRCDataToAttitudeManager(const RCControl &rcData);
        void sendRCDataToTelemetryManager(const RCControl &rcData);
        void sendHeartbeatDataToTelemetryManager(uint8_t baseMode, uint32_t customMode, MAV_STATE systemStatus);
        void sendMessagesToLogger();
};
