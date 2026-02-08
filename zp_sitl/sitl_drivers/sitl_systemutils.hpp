#pragma once
#include "systemutils_iface.hpp"
#include <chrono>
#include <thread>

class SITL_SystemUtils : public ISystemUtils {
private:
    std::chrono::steady_clock::time_point startTime;
    
public:
    SITL_SystemUtils() : startTime(std::chrono::steady_clock::now()) {}
    
    void delayMs(uint32_t delay_ms) override {
        std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
    }
    
    uint32_t getCurrentTimestampMs() override {
        auto now = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - startTime);
        return static_cast<uint32_t>(duration.count());
    }
};
