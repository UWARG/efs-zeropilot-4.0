#include "sd_threads.hpp"
#include "managers.hpp"
#include "utils.h"

osThreadId_t sdMainHandle;

static const osThreadAttr_t sdMainLoopAttr = {
    .name = "sdMain",
    .stack_size = 6144,
    .priority = (osPriority_t) osPriorityBelowNormal
};

void sdMainLoopWrapper(void *arg)
{
  while(true)
  {
    UBaseType_t freeWords = uxTaskGetStackHighWaterMark(NULL);
    ffmHandle->run();
  }
}

// Debug instrumentation: inspect these in the debugger after sdInitThreads().
// If sdMainHandle is NULL but sdHeapBeforeCreate is large, the failure is NOT heap.
volatile size_t sdHeapBeforeCreate = 0;
volatile size_t sdHeapAfterCreate  = 0;

void sdInitThreads()
{
    sdHeapBeforeCreate = xPortGetFreeHeapSize();
    sdMainHandle = osThreadNew(sdMainLoopWrapper, NULL, &sdMainLoopAttr);
    sdHeapAfterCreate = xPortGetFreeHeapSize();
    configASSERT(sdMainHandle != NULL);   // Halts here if creation fails
}
