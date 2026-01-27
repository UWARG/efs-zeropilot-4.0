#include "startup_threads.hpp"
#include "managers.hpp"
#include "utils.h"
#include "logger.hpp"

osThreadId_t startUpMainHandle;
extern Logger * loggerHandle;

static const osThreadAttr_t startUpAttr = {
    .name = "startUpMain",
    .stack_size = 1024,
    .priority = (osPriority_t) osPriorityHigh
};

void startUpMain(void *arg)
{
  loggerHandle->init();
  vTaskDelete(NULL);
}

void startUpInitThreads()
{
    startUpMainHandle = osThreadNew(startUpMain, NULL, &startUpAttr);
}
