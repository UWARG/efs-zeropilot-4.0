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
  while(true)
  {
    smHandle->smUpdate();
    osDelay(timeToTicks(SM_CONTROL_LOOP_DELAY));
  }
}

void smInitThreads()
{
    smMainHandle = osThreadNew(smMainLoopWrapper, NULL, &smMainLoopAttr);
}
