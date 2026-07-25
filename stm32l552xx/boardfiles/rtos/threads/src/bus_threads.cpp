#include "bus_threads.hpp"
#include "utils.h"
#include "drivers.hpp"
#include "FreeRTOS.h"
#include "main.h"

osThreadId_t busMainHandle = nullptr;

static StaticTask_t busMainControlBlock;
static StackType_t busMainStack[1024 / sizeof(StackType_t)];

static const osThreadAttr_t busMainLoopAttr = {
    .name = "busMain",
    .cb_mem = &busMainControlBlock,
    .cb_size = sizeof(busMainControlBlock),
    .stack_mem = busMainStack,
    .stack_size = sizeof(busMainStack),
    .priority = (osPriority_t) osPriorityNormal
};

void busMainLoopWrapper(void *arg)
{
  uint32_t nextWakeUp = osKernelGetTickCount();
  while(true)
  {
    if (canControllerHandle) {
      canControllerHandle->routineTasks();
    }

    nextWakeUp += timeToTicks(BUS_UPDATE_LOOP_DELAY_MS);
    osDelayUntil(nextWakeUp);
  }
}

void busInitThreads()
{
    busMainHandle = osThreadNew(busMainLoopWrapper, NULL, &busMainLoopAttr);

    if (busMainHandle == nullptr) {
        Error_Handler();
    }
}
