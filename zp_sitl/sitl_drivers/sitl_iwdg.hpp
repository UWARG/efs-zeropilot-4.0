#pragma once
#include "iwdg_iface.hpp"
#include <chrono>

#define WATCHDOG_TIMEOUT_MS 10000 // 10 seconds

class SITL_IWDG : public IIndependentWatchdog {
private:
    uint32_t timeoutCounter = 0;

public:
    bool check_watchdog() {
        timeoutCounter++;
        return timeoutCounter < WATCHDOG_TIMEOUT_MS;
    }

    bool refreshWatchdog() override { 
        timeoutCounter = 0; 
        return true;
    }
};
