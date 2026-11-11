#include "tm_threads.hpp"
#include "managers.hpp"
#include "utils.h"

osThreadId_t tmMainHandle;

static const osThreadAttr_t tmMainLoopAttr = {
    .name = "tmMain",
    .stack_size = 4096,
    .priority = (osPriority_t) osPriorityNormal
};

void tmMainLoopWrapper(void *arg)
{
  uint32_t nextWakeUp = osKernelGetTickCount();
  while(true)
  {
    tmHandle->tmUpdate();
    nextWakeUp += timeToTicks(TM_UPDATE_LOOP_DELAY_MS);
    osDelayUntil(nextWakeUp);
  }
}

void tmInitThreads()
{
    tmMainHandle = osThreadNew(tmMainLoopWrapper, NULL, &tmMainLoopAttr);
}
