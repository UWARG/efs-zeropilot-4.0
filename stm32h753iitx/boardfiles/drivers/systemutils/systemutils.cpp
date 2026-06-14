#include "cmsis_os.h"
#include "systemutils.hpp"
#include "stm32h7xx_hal.h"

struct TaskEntry {
    const char* name;
    uint32_t startCycle;
    uint32_t deltaUs;
    uint32_t deltaStartTime;
};

static TaskEntry registry[MAX_PROFILED_TASKS] = {};
static uint8_t taskCount = 0;

void SystemUtils::delayMs(uint32_t delay_ms) {
    HAL_Delay(delay_ms);
}

uint32_t SystemUtils::getCurrentTimestampMs() {
    return (osKernelGetTickCount() * 1000) / osKernelGetTickFreq();
}


void SystemUtils::dwtInit() {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void SystemUtils::profilerRegister(const char* name, uint8_t* outId) {
    if (taskCount == 0) dwtInit();
    if (taskCount >= MAX_PROFILED_TASKS) {
        *outId = -1;
        return;
    }
    uint8_t id = taskCount++;
    registry[id] = { name, 0, 0, 0 };
    *outId = id;
}

void SystemUtils::profilerBegin(uint8_t id) {
    uint32_t cycles = DWT->CYCCNT - registry[id].startCycle;
    registry[id].deltaStartTime = cycles / (SystemCoreClock / 1000000U);
    registry[id].startCycle = DWT->CYCCNT;
}

void SystemUtils::profilerEnd(uint8_t id) {
    uint32_t cycles = DWT->CYCCNT - registry[id].startCycle;
    registry[id].deltaUs = cycles / (SystemCoreClock / 1000000U); // convert hclk ticks to micro sec
}

void SystemUtils::profilerGetAll(TaskProfile* out, uint8_t* count) {
    *count = taskCount;
    for (uint8_t i = 0; i < taskCount; i++) {
        out[i] = { registry[i].name, registry[i].deltaUs, registry[i].deltaStartTime > 0 ? 1000000U / registry[i].deltaStartTime : 0 };
    }
}
