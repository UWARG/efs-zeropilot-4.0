#include "cmsis_os.h"
#include "systemutils.hpp"
#include "arm_math.h"

struct TaskEntry {
    const char* name;
    uint32_t startCycle;
    uint32_t maxExecUs;   // worst-case exec time since last profilerGetAll
    uint32_t iterations;  // completed begin/end pairs since last profilerGetAll
};

static TaskEntry registry[MAX_PROFILED_TASKS] = {};
static uint8_t taskCount = 0;
static uint32_t lastReadCycle = 0;
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
    registry[id].startCycle = DWT->CYCCNT;
}

void SystemUtils::profilerEnd(uint8_t id) {
    uint32_t cycles = DWT->CYCCNT - registry[id].startCycle;
    uint32_t execUs = cycles / (SystemCoreClock / 1000000U); // convert hclk ticks to micro sec
    if (execUs > registry[id].maxExecUs) {
        registry[id].maxExecUs = execUs;
    }
    registry[id].iterations++;
}

// deltaExec = worst-case exec time (us) and deltaPeriod = average rate (hz) since the
// previous call. Must be called more often than the DWT wrap (2^32 cycles, ~9 s at 480 MHz).
void SystemUtils::profilerGetAll(TaskProfile* out, uint8_t* count) {
    uint32_t now = DWT->CYCCNT;
    uint32_t windowCycles = now - lastReadCycle;
    lastReadCycle = now;

    *count = taskCount;
    for (uint8_t i = 0; i < taskCount; i++) {
        uint32_t rateHz = 0;
        if (windowCycles > 0) {
            rateHz = (uint64_t)registry[i].iterations * SystemCoreClock / windowCycles;
        }
        out[i] = { registry[i].name, registry[i].maxExecUs, rateHz };
        registry[i].maxExecUs = 0;
        registry[i].iterations = 0;
    }
}

// Straight dividing DWT->CYCCNT by (SystemCoreClock / 1000000) ruins uint32_t modular arithmetic 
// DWT->CYCCNT is 32 bit counter and wraps perfectly but it is divided after the wrap, making it not wrapping 32 bit anymore. wraps at 2^32/10^6 = 9s
// This causes a mismatch between the wrapping of the value and the uint32_t arithmetic
// So flip the order here, divided then mod instead of mod then divide
// Calculate the cycle count in 64 bits (big enough so DWT dont wrap in 32 bits), divide to make it in microsec, then let uint32_t cast it, so the us value wraps at 2^32 = 71 mins
uint32_t SystemUtils::getDWTMicroSec() {
    const uint32_t PRIMASK = __get_PRIMASK();
    __disable_irq();

    uint32_t now = DWT->CYCCNT;
    microSecAccumCyc += (uint32_t)(now - microSecLastCyc);
    microSecLastCyc = now;

    __set_PRIMASK(PRIMASK);

    return (uint32_t)(microSecAccumCyc / (SystemCoreClock / 1000000U));
}

float SystemUtils::dspSinf(float x) {
    return arm_sin_f32(x);
}

float SystemUtils::dspCosf(float x) {
    return arm_cos_f32(x);
}
