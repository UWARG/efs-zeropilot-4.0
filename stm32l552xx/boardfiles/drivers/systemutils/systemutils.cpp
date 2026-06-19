#include "cmsis_os.h"
#include "systemutils.hpp"
#include "stm32l5xx_hal.h"

struct TaskEntry {
    const char* name;
    uint32_t startCycle;
    uint32_t deltaUs;
    uint32_t deltaStartTime;
};

static TaskEntry registry[MAX_PROFILED_TASKS] = {};
static uint8_t taskCount = 0;

ZP_ERROR_e SystemUtils::delayMs(uint32_t delay_ms) {
    HAL_Delay(delay_ms);
    return ZP_ERROR_OK;
}

ZP_ERROR_e SystemUtils::getCurrentTimestampMs(uint32_t& currentTime) {
    currentTime = (osKernelGetTickCount() * 1000) / osKernelGetTickFreq();
    return ZP_ERROR_OK;
}


ZP_ERROR_e SystemUtils::dwtInit() {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    return ZP_ERROR_OK;
}

ZP_ERROR_e SystemUtils::profilerRegister(const char* name, uint8_t* outId) {
    if (taskCount == 0) dwtInit();
    if (taskCount >= MAX_PROFILED_TASKS) {
        *outId = -1;
        return ZP_ERROR_OUT_OF_MEMORY;
    }
    uint8_t id = taskCount++;
    registry[id] = { name, 0, 0, 0 };
    *outId = id;
    return ZP_ERROR_OK;
}

ZP_ERROR_e SystemUtils::profilerBegin(uint8_t id) {
    uint32_t cycles = DWT->CYCCNT - registry[id].startCycle;
    registry[id].deltaStartTime = cycles / (SystemCoreClock / 1000000U);
    registry[id].startCycle = DWT->CYCCNT;
    return ZP_ERROR_OK;
}

ZP_ERROR_e SystemUtils::profilerEnd(uint8_t id) {
    uint32_t cycles = DWT->CYCCNT - registry[id].startCycle;
    registry[id].deltaUs = cycles / (SystemCoreClock / 1000000U); // convert hclk ticks to micro sec
    return ZP_ERROR_OK;
}

ZP_ERROR_e SystemUtils::profilerGetAll(TaskProfile* out, uint8_t* count) {
    *count = taskCount;
    for (uint8_t i = 0; i < taskCount; i++) {
        out[i] = { registry[i].name, registry[i].deltaUs, registry[i].deltaStartTime > 0 ? 1000000U / registry[i].deltaStartTime : 0 };
    }
    return ZP_ERROR_OK;
}
