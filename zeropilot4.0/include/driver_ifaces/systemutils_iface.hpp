#pragma once

#include <cstdint>

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

        virtual void delayMs(uint32_t delay_ms) = 0;
        virtual uint32_t getCurrentTimestampMs() = 0;

        virtual void profilerRegister(const char* name, uint8_t* outId) = 0;
        virtual void profilerBegin(uint8_t id) = 0;
        virtual void profilerEnd(uint8_t id) = 0;
        virtual void profilerGetAll(TaskProfile* out, uint8_t* count) = 0;
};
