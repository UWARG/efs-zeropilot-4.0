#include "sm_threads.hpp"
#include "managers.hpp"
#include "can_controller.hpp"
#include "utils.h"

extern CANController *canControllerHandle;

osThreadId_t smMainHandle;

static const osThreadAttr_t smMainLoopAttr = {
    .name = "smMain",
    .stack_size = 2048,
    .priority = (osPriority_t) osPriorityNormal
};

void smMainLoopWrapper(void *arg)
{
  uint32_t nextWakeUp = osKernelGetTickCount();
  while(true)
  {
    smHandle->smUpdate();
    if (canControllerHandle) {
      canControllerHandle->routineTasks();
    }
    nextWakeUp += timeToTicks(SM_UPDATE_LOOP_DELAY_MS);
    osDelayUntil(nextWakeUp);
  }
}

void smInitThreads()
{
    smMainHandle = osThreadNew(smMainLoopWrapper, NULL, &smMainLoopAttr);
}
