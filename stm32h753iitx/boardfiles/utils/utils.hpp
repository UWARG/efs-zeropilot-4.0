#pragma once

#include "cmsis_os.h"
#include "stm32h7xx_hal.h"

#ifdef __cplusplus
#include "zp_error.h"

extern "C" {
#endif // __cplusplus

uint32_t timeToTicks(uint32_t duration_ms);

#ifdef __cplusplus
}
#endif // __cplusplus

ZP_Error halToZpError(HAL_StatusTypeDef status);
