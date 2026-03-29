#include "sd_threads.hpp"
#include "managers.hpp"
#include "utils.h"

osThreadId_t sdMainHandle;

static const osThreadAttr_t sdMainLoopAttr = {
    .name = "sdMain",
    .stack_size = 1024,
    .priority = (osPriority_t) osPriorityBelowNormal
};

void sdMainLoopWrapper(void *arg)
{
  while(true)
  {
    ffmHandle->run();
    osDelay(timeToTicks(SD_UPDATE_LOOP_DELAY_MS));
  }
}

void sdInitThreads()
{
    sdMainHandle = osThreadNew(sdMainLoopWrapper, NULL, &sdMainLoopAttr);
}
