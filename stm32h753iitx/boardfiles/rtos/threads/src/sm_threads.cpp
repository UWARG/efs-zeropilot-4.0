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
  uint32_t nextWakeUp = osKernelGetTickCount();
  while(true)
  {
    smHandle->smUpdate();
    nextWakeUp += timeToTicks(SM_UPDATE_LOOP_DELAY_MS);
    osDelayUntil(nextWakeUp);
  }
}

void smInitThreads()
{
    smMainHandle = osThreadNew(smMainLoopWrapper, NULL, &smMainLoopAttr);
}
