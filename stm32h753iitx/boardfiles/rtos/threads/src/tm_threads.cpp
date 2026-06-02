#include "tm_threads.hpp"
#include "systemutils.hpp"
#include "managers.hpp"
#include "utils.h"

osThreadId_t tmMainHandle;

static const osThreadAttr_t tmMainLoopAttr = {
    .name = "tmMain",
    .stack_size = 2048,
    .priority = (osPriority_t) osPriorityNormal
};

void tmMainLoopWrapper(void *arg)
{
  uint8_t profileId;
  SystemUtils::profilerRegister("TM", &profileId);

  while(true)
  {
    SystemUtils::profilerBegin(profileId);
    tmHandle->tmUpdate();
    SystemUtils::profilerEnd(profileId);
    osDelay(timeToTicks(TM_UPDATE_LOOP_DELAY_MS));
  }
}

void tmInitThreads()
{
    tmMainHandle = osThreadNew(tmMainLoopWrapper, NULL, &tmMainLoopAttr);
}
