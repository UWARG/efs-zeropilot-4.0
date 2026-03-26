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
    amHandle->amUpdate();
    osDelay(timeToTicks(AM_UPDATE_LOOP_DELAY_MS));
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
