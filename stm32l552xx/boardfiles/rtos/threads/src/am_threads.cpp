#include "am_threads.hpp"
#include "systemutils.hpp"
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
  uint8_t profileId;
  SystemUtils::profilerRegister("amMain", &profileId);

  while(true)
  {
    SystemUtils::profilerBegin(profileId);
    amHandle->amUpdate();
    SystemUtils::profilerEnd(profileId);
    osDelay(timeToTicks(AM_UPDATE_LOOP_DELAY_MS));
  }
}

void amInitThreads()
{
    amMainHandle = osThreadNew(amMainLoopWrapper, NULL, &amMainLoopAttr);
}
