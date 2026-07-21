#include "am_threads.hpp"
#include "managers.hpp"
#include "utils.h"

osThreadId_t amMainHandle;

static const osThreadAttr_t amMainLoopAttr = {
    .name = "amMain",
    .stack_size = 4096,
    .priority = (osPriority_t) osPriorityNormal
};

void amMainLoopWrapper(void *arg)
{
  uint32_t nextWakeUp = osKernelGetTickCount();
  while(true)
  {
    amHandle->amUpdate();
    nextWakeUp += timeToTicks(AM_UPDATE_LOOP_DELAY_MS);
    osDelayUntil(nextWakeUp);
  }
}

void amInitThreads()
{
    amMainHandle = osThreadNew(amMainLoopWrapper, NULL, &amMainLoopAttr);
}
