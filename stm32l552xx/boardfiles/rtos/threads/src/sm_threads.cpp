#include "sm_threads.hpp"
#include "task_profiler.hpp"
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
  uint8_t profileId;
  profilerRegister("smMain", &profileId);

  while(true)
  {
    profilerBegin(profileId);
    smHandle->smUpdate();
    profilerEnd(profileId);
    osDelay(timeToTicks(SM_UPDATE_LOOP_DELAY_MS));
  }
}

void smInitThreads()
{
    smMainHandle = osThreadNew(smMainLoopWrapper, NULL, &smMainLoopAttr);
}
