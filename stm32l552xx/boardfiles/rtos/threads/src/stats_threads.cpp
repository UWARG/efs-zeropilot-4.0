#include "stats_thread.hpp"
#include "task_profiler.hpp"
#include "utils.h"
#include "main.h"
#include <cstring>
#include <cstdio>

osThreadId_t statsMainHandle;

static const osThreadAttr_t statsMainLoopAttr = {
    .name = "statsMain",
    .stack_size = 1024,
    .priority = (osPriority_t) osPriorityNormal
};

void statsMainLoop(void *arg)
{
  static uint8_t buf[256];
  static TaskProfile profiles[MAX_PROFILED_TASKS];

  while(true)
  {
    osDelay(timeToTicks(2000));

    uint8_t count = 0;
    profilerGetAll(profiles, &count);

    uint16_t offset = 0;
    offset += snprintf((char*)buf + offset, sizeof(buf) - offset, "--- task rates ---\r\n");
    for (uint8_t i = 0; i < count && offset < sizeof(buf) - 1; i++) {
      offset += snprintf((char*)buf + offset, sizeof(buf) - offset,
                         "%-12s %lu ms\r\n", profiles[i].name, profiles[i].delta);
    }

    HAL_UART_AbortTransmit(&huart1);
    HAL_UART_Transmit_DMA(&huart1, buf, offset);
  }
}

void statsInitThread() {
    statsMainHandle = osThreadNew(statsMainLoop, NULL, &statsMainLoopAttr);
}
