#pragma once

#include "stm32l5xx_hal.h"
#include "zp_error.h"

inline ZP_Error halToZpError(HAL_StatusTypeDef status) {
    if (status == HAL_OK) {
        return ZP_ERROR_OK;
    } else if (status == HAL_TIMEOUT) {
        return ZP_ERROR_EXT_API | ZP_ERROR_TIMEOUT;
    } else if (status == HAL_BUSY) {
        return ZP_ERROR_EXT_API | ZP_ERROR_BUSY;
    }
    return ZP_ERROR_EXT_API | ZP_ERROR_FAIL;
}
