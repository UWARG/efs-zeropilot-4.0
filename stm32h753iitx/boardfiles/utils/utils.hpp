#pragma once

#include "cmsis_os.h"
#include "stm32h7xx_hal.h"

#ifdef __cplusplus
#include "zp_error.h"

extern "C" {
#endif

uint32_t timeToTicks(uint32_t duration_ms);

#ifdef __cplusplus
}

inline ZP_Error halToZpError(HAL_StatusTypeDef status);

#endif
