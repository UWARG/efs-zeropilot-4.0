#pragma once

#include "cmsis_os.h"

#ifdef __cplusplus
#include "stm32l5xx_hal.h"
#include "zp_error.h"
extern "C" {
#endif

uint32_t timeToTicks(uint32_t duration_ms);

#ifdef __cplusplus
}

// C++ only, since ZP_Error is a C++ class. C files never see this.
static inline ZP_Error halStatusToError(HAL_StatusTypeDef status) {
    if (status == HAL_OK) {
        return ZP_ERROR_OK;
    } else if (status == HAL_TIMEOUT) {
        return ZP_ERROR_EXT_API | ZP_ERROR_TIMEOUT;
    } else if (status == HAL_BUSY) {
        return ZP_ERROR_EXT_API | ZP_ERROR_BUSY;
    }
    return ZP_ERROR_EXT_API | ZP_ERROR_FAIL;
}
#endif
