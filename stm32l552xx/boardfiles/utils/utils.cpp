#include "utils.hpp"

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus
  
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

#ifdef __cplusplus
}
#endif // __cplusplus
