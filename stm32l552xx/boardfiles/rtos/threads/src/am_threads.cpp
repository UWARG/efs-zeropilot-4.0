#include "am_threads.hpp"
#include "managers.hpp"
#include "utils.h"

osThreadId_t amMainHandle;

static const osThreadAttr_t amMainLoopAttr = {
    .name = "amMain",
    .stack_size = 1024,
    .priority = (osPriority_t) osPriorityNormal
};

void amMainLoopWrapper(void *arg)
{
  while(true)
  {
    amHandle->runControlLoopIteration();
    uint32_t ticks = 0;
    if (timeToTicks(&ticks, 50) == ZP_ERROR_OK) {
      osDelay(ticks);
    }
  }
}

ZP_ERROR_e amInitThreads()
{
    amMainHandle = osThreadNew(amMainLoopWrapper, NULL, &amMainLoopAttr);
    if (amMainHandle == NULL) {
        return ZP_ERROR_FAIL;
    }
    return ZP_ERROR_OK;
}
