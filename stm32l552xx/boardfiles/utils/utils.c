#include "utils.h"

ZP_ERROR_e timeToTicks(uint32_t *ticks, uint32_t duration_ms)
{
  if (ticks == NULL) {
    return ZP_ERROR_NULLPTR;
  }

  uint32_t calculated_ticks = (osKernelGetTickFreq() * duration_ms) / 1000U;
  if(calculated_ticks > 1U)
  {
    *ticks = calculated_ticks;
  }
  else
  {
    *ticks = 1U;
  }

  return ZP_ERROR_OK;
}
