#pragma once

#include <cstdint>
#include "zp_error.h"

#define MAX_PROFILED_TASKS 4

struct TaskProfile {
    const char* name;
    uint32_t deltaExec;
    uint32_t deltaPeriod;
};

class ISystemUtils {
    protected:
        ISystemUtils() = default;

    public:
        virtual ~ISystemUtils() = default;

        virtual ZP_ERROR_e delayMs(uint32_t delay_ms) = 0;
        virtual ZP_ERROR_e getCurrentTimestampMs(uint32_t& currentTime) = 0;

        virtual ZP_ERROR_e profilerRegister(const char* name, uint8_t* outId) = 0;
        virtual ZP_ERROR_e profilerBegin(uint8_t id) = 0;
        virtual ZP_ERROR_e profilerEnd(uint8_t id) = 0;
        virtual ZP_ERROR_e profilerGetAll(TaskProfile* out, uint8_t* count) = 0;
};
