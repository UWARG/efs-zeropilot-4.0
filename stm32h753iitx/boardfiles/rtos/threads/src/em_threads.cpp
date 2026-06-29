#include "em_threads.hpp"
#include "managers.hpp"
#include "utils.h"

osThreadId_t emMainHandle;

static const osThreadAttr_t sdMainLoopAttr = {
    .name = "emMain",
    .stack_size = 6144,
    .priority = (osPriority_t) osPriorityBelowNormal
};

void emMainLoopWrapper(void *arg)
{
  while(true)
  {
    emHandle->run();
  }
}

void emInitThreads()
{
    emMainHandle = osThreadNew(emMainLoopWrapper, NULL, &sdMainLoopAttr);
}
