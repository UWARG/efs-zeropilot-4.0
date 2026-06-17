#pragma once

#include "cmsis_os.h"
#include "zp_error.h"

#ifdef __cplusplus
extern "C" {
#endif

uint32_t timeToTicks(uint32_t duration_ms);

#ifdef __cplusplus
}
#endif
