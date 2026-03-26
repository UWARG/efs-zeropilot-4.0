#pragma once

#include "cmsis_os.h"
#include "error.h"

#ifdef __cplusplus
extern "C" {
#endif

ZP_ERROR_e timeToTicks(uint32_t *ticks, uint32_t duration_ms);

#ifdef __cplusplus
}
#endif
