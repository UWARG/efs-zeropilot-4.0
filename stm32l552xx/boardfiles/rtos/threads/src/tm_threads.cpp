#include "tm_threads.hpp"
#include "managers.hpp"
#include "utils.h"

osThreadId_t tmMainHandle;

static const osThreadAttr_t tmMainLoopAttr = {
    .name = "tmMain",
    .stack_size = 4096,
    .priority = (osPriority_t) osPriorityNormal
};

void tmMainLoopWrapper(void *arg)
{
  while(true)
  {
    tmHandle->tmUpdate();
    osDelay(timeToTicks(TM_UPDATE_LOOP_DELAY_MS));
  }
}

ZP_ERROR_e tmInitThreads()
{
    tmMainHandle = osThreadNew(tmMainLoopWrapper, NULL, &tmMainLoopAttr);
    if (tmMainHandle == NULL) {
        return ZP_ERROR_FAIL;
    }
    return ZP_ERROR_OK;
}
