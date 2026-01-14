#include "cmsis_os.h"
#include "systemutils.hpp"
#include "stm32h7xx_hal.h"

void SystemUtils::delayMs(uint32_t delay_ms) {
    HAL_Delay(delay_ms);
}

uint32_t SystemUtils::getCurrentTimestampMs() {
    return (osKernelGetTickCount() * 1000) / osKernelGetTickFreq();
}
