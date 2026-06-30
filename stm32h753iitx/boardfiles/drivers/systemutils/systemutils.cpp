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
static uint32_t microSecLastCyc = 0;
static uint64_t microSecAccumCyc = 0;

void SystemUtils::delayMs(uint32_t delay_ms) {
    HAL_Delay(delay_ms);
}

uint32_t SystemUtils::getCurrentTimestampMs() {
    return (osKernelGetTickCount() * 1000) / osKernelGetTickFreq();
}

void SystemUtils::dwtInit() {
    if (DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) return; // DWT is already running
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

// Straight dividing DWT->CYCCNT by (SystemCoreClock / 1000000) ruins uint32_t modular arithmetic 
// DWT->CYCCNT is 32 bit counter and wraps perfectly but it is divided after the wrap, making it not wrapping 32 bit anymore. wraps at 2^32/10^6 = 9s
// This causes a mismatch between the wrapping of the value and the uint32_t arithmetic
// So flip the order here, divided then mod instead of mod then divide
// Calculate the cycle count in 64 bits (big enough so DWT dont wrap in 32 bits), divide to make it in microsec, then let uint32_t cast it, so the us value wraps at 2^32 = 71 mins
uint32_t SystemUtils::getDWTMicroSec() {
    uint32_t now = DWT->CYCCNT;
    microSecAccumCyc += (uint32_t)(now - microSecLastCyc);
    microSecLastCyc = now;
    return (uint32_t)(microSecAccumCyc / (SystemCoreClock / 1000000U));
}