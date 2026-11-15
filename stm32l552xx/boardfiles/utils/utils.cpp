#include "utils.hpp"

ZP_Error halToZpError(HAL_StatusTypeDef status) {
  if (status == HAL_OK) {
    return ZP_ERROR_OK;
  } else if (status == HAL_TIMEOUT) {
    return ZP_ERROR_EXT_API | ZP_ERROR_TIMEOUT;
  } else if (status == HAL_BUSY) {
    return ZP_ERROR_EXT_API | ZP_ERROR_BUSY;
  }
  return ZP_ERROR_EXT_API | ZP_ERROR_FAIL;
}

#ifdef __cplusplus

uint32_t timeToTicks(uint32_t duration_ms)
{
  uint32_t ticks = (osKernelGetTickFreq() * duration_ms) / 1000U;
  if(ticks > 1U)
  {
    return ticks;
  }
  else
  {
    return 1U;
  }
}

#endif // __cplusplus
