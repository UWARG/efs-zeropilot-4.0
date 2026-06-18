#pragma once
#include "systemutils_iface.hpp"
#include <chrono>
#include <thread>

class SITL_SystemUtils : public ISystemUtils {
private:
    std::chrono::steady_clock::time_point startTime;
    
public:
    SITL_SystemUtils() : startTime(std::chrono::steady_clock::now()) {}
    
    ZP_ERROR_e delayMs(uint32_t delay_ms) override {
        std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
        return ZP_ERROR_OK;
    }
    
    ZP_ERROR_e getCurrentTimestampMs(uint32_t& currentTime) override {
        auto now = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - startTime);
        currentTime = static_cast<uint32_t>(duration.count());
        return ZP_ERROR_OK;
    }

    ZP_ERROR_e profilerRegister(const char* name, uint8_t* outId) override { return ZP_ERROR_OK; }

    ZP_ERROR_e profilerBegin(uint8_t id) override { return ZP_ERROR_OK; }

    ZP_ERROR_e profilerEnd(uint8_t id) override { return ZP_ERROR_OK; }

    ZP_ERROR_e profilerGetAll(TaskProfile* out, uint8_t* count) override { return ZP_ERROR_OK; }
};
