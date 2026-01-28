#pragma once

#include "iwdg_iface.hpp"
#include "systemutils_iface.hpp"
#include "mavlink.h"
#include "logger.hpp"
#include "config.hpp"
#include "queue_iface.hpp"
#include "rc_iface.hpp"
#include "rc_motor_control.hpp"
#include "config_msg.hpp"
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
            IRCReceiver *rcDriver,
			IPowerModule *pmDriver,
            IMessageQueue<RCMotorControlMessage_t> *amRCQueue,
            IMessageQueue<TMMessage_t> *tmQueue,
            IMessageQueue<TMSMMessage_t> *tmSmQueue,
            IMessageQueue<char[100]> *smLoggerQueue,
            IMessageQueue<ConfigMessage_t> **smConfigRouteQueue,
            Logger *logger,
            Config *config
        );

        void smUpdate(); // This function is the main function of SM, it should be called in the main loop of the system.

    private:
        IIndependentWatchdog *iwdgDriver; // Independent Watchdog driver

        ISystemUtils *systemUtilsDriver; // System utilities instance
        Logger *logger; // Logger 
        Config *config; // Config manager
        IRCReceiver *rcDriver; // RC receiver driver
        IPowerModule *pmDriver;
        
        IMessageQueue<RCMotorControlMessage_t> *amRCQueue; // Queue driver for tx communication to the Attitude Manager
        IMessageQueue<TMMessage_t> *tmQueue; // Queue driver for tx communication to the Telemetry Manager
        IMessageQueue<TMSMMessage_t> *tmSmQueue; // Queue driver for rx communication from Telemetry Manager to System Manager
        IMessageQueue<char[100]> *smLoggerQueue; // Queue driver for rx communication from other modules to the System Manager for logging
        IMessageQueue<ConfigMessage_t> *smConfigRouteQueue[static_cast<size_t>(Owner_e::COUNT)] = {}; // Array of queues for routing config messages to respective managers

        uint8_t smSchedulingCounter;
        int paramAmountSent = -1; // Counter to keep track of how many params have been sent to TM

        void sendRCDataToAttitudeManager(const RCControl &rcData);
        void sendRCDataToTelemetryManager(const RCControl &rcData);
        void sendHeartbeatDataToTelemetryManager(uint8_t baseMode, uint32_t customMode, MAV_STATE systemStatus);
        void sendParamDataToTelemetryManager();
        void sendMessagesToLogger();
        void handleMessagesFromTelemetryManager();
};
