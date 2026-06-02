#include "task_profiler.hpp"
#include "stm32l5xx.h"

struct TaskEntry {
    const char* name;
    uint32_t startCycle;
    uint32_t deltaUs;
};

static TaskEntry registry[MAX_PROFILED_TASKS] = {};
static uint8_t taskCount = 0;

static void dwtInit() {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void profilerRegister(const char* name, uint8_t* outId) {
    if (taskCount == 0) dwtInit();
    if (taskCount >= MAX_PROFILED_TASKS) return;
    uint8_t id = taskCount++;
    registry[id] = { name, 0, 0 };
    *outId = id;
}

void profilerBegin(uint8_t id) {
    registry[id].startCycle = DWT->CYCCNT;
}

void profilerEnd(uint8_t id) {
    uint32_t cycles = DWT->CYCCNT - registry[id].startCycle;
    registry[id].deltaUs = cycles / (SystemCoreClock / 1000000U); // convert hclk ticks to micro sec
}

void profilerGetAll(TaskProfile* out, uint8_t* count) {
    *count = taskCount;
    for (uint8_t i = 0; i < taskCount; i++) {
        out[i] = { registry[i].name, registry[i].deltaUs };
    }
}
