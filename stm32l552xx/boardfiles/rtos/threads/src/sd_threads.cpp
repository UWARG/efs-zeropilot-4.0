#include "sd_threads.hpp"
#include "managers.hpp"
#include "utils.h"
#include "museq.hpp"
#include "cmsis_os.h"

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
    SdReqMsg reqMsg;
    if (osMessageQueueGet(sdRequestQueueId, &reqMsg, NULL, osWaitForever) == osOK) {
      sdmHandle->sdUpdate(reqMsg);
    }
  }
}

void sdInitThreads()
{
    sdMainHandle = osThreadNew(sdMainLoopWrapper, NULL, &sdMainLoopAttr);
}
