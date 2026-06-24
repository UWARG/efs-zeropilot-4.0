#include "bus_threads.hpp"
#include "utils.h"
#include "drivers.hpp"

osThreadId_t busMainHandle;

static const osThreadAttr_t busMainLoopAttr = {
    .name = "busMain",
    .stack_size = 1024,
    .priority = (osPriority_t) osPriorityNormal
};

void busMainLoopWrapper(void *arg)
{
  uint32_t nextWakeUp = osKernelGetTickCount();
  while(true)
  {
    bool success = canControllerHandle->routineTasks();
    (void)success;

    nextWakeUp += timeToTicks(BUS_UPDATE_LOOP_DELAY_MS);
    osDelayUntil(nextWakeUp);
  }
}

void busInitThreads()
{
    busMainHandle = osThreadNew(busMainLoopWrapper, NULL, &busMainLoopAttr);
}
