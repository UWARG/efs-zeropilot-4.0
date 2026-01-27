#include "sm_threads.hpp"
#include "managers.hpp"
#include "utils.h"

osThreadId_t smMainHandle;

static const osThreadAttr_t smMainLoopAttr = {
    .name = "smMain",
    .stack_size = 1024,
    .priority = (osPriority_t) osPriorityNormal
};

void smMainLoopWrapper(void *arg)
{
  ZP_ERROR_e err;
  while(true)
  {
    err = smHandle->smUpdate();
    // TODO: Handle error appropriately (e.g., log, enter safe mode, etc.)
    (void)err; // Suppress unused variable warning for now
    uint32_t ticks = 0;
    if (timeToTicks(&ticks, 50) == ZP_ERROR_OK) {
      osDelay(ticks);
    }
  }
}

ZP_ERROR_e smInitThreads()
{
    smMainHandle = osThreadNew(smMainLoopWrapper, NULL, &smMainLoopAttr);
    if (smMainHandle == NULL) {
        return ZP_ERROR_FAIL;
    }
    return ZP_ERROR_OK;
}
