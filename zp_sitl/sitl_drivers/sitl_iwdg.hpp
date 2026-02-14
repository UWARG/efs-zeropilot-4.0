#pragma once
#include "iwdg_iface.hpp"
#include "sitl_driver_configs.hpp"
#include <chrono>

class SITL_IWDG : public IIndependentWatchdog {
private:
    using Config = SITL_Driver_Configs::SITL_IWDG_Config;

    uint32_t timeoutCounter = 0;

public:
    bool check_watchdog() {
        timeoutCounter += Config::WATCHDOG_CHECK_INTERVAL_MS;
        return timeoutCounter < Config::WATCHDOG_TIMEOUT_MS;
    }

    bool refreshWatchdog() override { 
        timeoutCounter = 0; 
        return true;
    }
};
