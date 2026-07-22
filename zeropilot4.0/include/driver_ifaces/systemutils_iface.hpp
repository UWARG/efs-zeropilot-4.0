#pragma once

#include <cstdint>

#define MAX_PROFILED_TASKS 4

struct TaskProfile {
    const char* name;
    uint32_t maxExecUs;
    uint32_t avgRateHz;
};

class ISystemUtils {
    protected:
        ISystemUtils() = default;

    public:
        virtual ~ISystemUtils() = default;

        virtual void delayMs(uint32_t delay_ms) = 0;
        virtual uint32_t getCurrentTimestampMs() = 0;

        virtual void profilerRegister(const char* name, uint8_t* outId) = 0;
        virtual void profilerBegin(uint8_t id) = 0;
        virtual void profilerEnd(uint8_t id) = 0;
        virtual void profilerGetAll(TaskProfile* out, uint8_t* count) = 0;

        virtual float dspSinf(float x) = 0;
        virtual float dspCosf(float x) = 0;
};
