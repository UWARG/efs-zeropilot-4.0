#include "startup_threads.hpp"
#include "managers.hpp"
#include "utils.h"
#include "logger.hpp"
#include "error.h"

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

ZP_ERROR_e startUpInitThreads()
{
    startUpMainHandle = osThreadNew(startUpMain, NULL, &startUpAttr);
    if (startUpMainHandle == NULL) {
        return ZP_ERROR_FAIL;
    }
    return ZP_ERROR_OK;
}
