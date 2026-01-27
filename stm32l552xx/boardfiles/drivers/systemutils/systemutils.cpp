#include "cmsis_os.h"
#include "systemutils.hpp"
#include "stm32l5xx_hal.h"

ZP_ERROR_e SystemUtils::delayMs(uint32_t delay_ms) {
    HAL_Delay(delay_ms);
    return ZP_ERROR_OK;
}

ZP_ERROR_e SystemUtils::getCurrentTimestampMs(uint32_t *timestamp_ms) {
    if (timestamp_ms == NULL) {
        return ZP_ERROR_NULLPTR;
    }

    *timestamp_ms = (osKernelGetTickCount() * 1000) / osKernelGetTickFreq();
    return ZP_ERROR_OK;
}
