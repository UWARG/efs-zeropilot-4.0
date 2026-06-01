#include "task_profiler.hpp"

struct TaskEntry {
    const char* name;
    uint32_t lastTick;
    uint32_t delta;
};

static TaskEntry registry[MAX_PROFILED_TASKS] = {};
static uint8_t taskCount = 0;

void profilerRegister(const char* name, uint8_t* outId) {
    if (taskCount >= MAX_PROFILED_TASKS) return;
    uint8_t id = taskCount++;
    registry[id] = { name, osKernelGetTickCount(), 0 };
    *outId = id;
}

void profilerTick(uint8_t id) {
    uint32_t now = osKernelGetTickCount();
    registry[id].delta = now - registry[id].lastTick;
    registry[id].lastTick = now;
}

void profilerGetAll(TaskProfile* out, uint8_t* count) {
    *count = taskCount;
    for (uint8_t i = 0; i < taskCount; i++) {
        out[i] = { registry[i].name, registry[i].delta };
    }
}
