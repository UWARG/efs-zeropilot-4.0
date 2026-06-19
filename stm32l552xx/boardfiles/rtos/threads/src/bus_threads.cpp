#include "bus_threads.hpp"
#include "can_controller.hpp"
#include "utils.h"
#include "FreeRTOS.h"
#include "main.h"

extern CANController *canControllerHandle;

osThreadId_t busMainHandle = nullptr;
volatile uint32_t busMainLoopCount = 0;
volatile uint32_t busFreeHeapBeforeCreate = 0;
volatile uint32_t busFreeHeapAfterCreate = 0;

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
    busMainLoopCount++;
    if (canControllerHandle) {
      canControllerHandle->routineTasks();
    }

    nextWakeUp += timeToTicks(BUS_UPDATE_LOOP_DELAY_MS);
    osDelayUntil(nextWakeUp);
  }
}

void busInitThreads()
{
    busFreeHeapBeforeCreate = static_cast<uint32_t>(xPortGetFreeHeapSize());
    busMainHandle = osThreadNew(busMainLoopWrapper, NULL, &busMainLoopAttr);
    busFreeHeapAfterCreate = static_cast<uint32_t>(xPortGetFreeHeapSize());

    if (busMainHandle == nullptr) {
        Error_Handler();
    }
}
