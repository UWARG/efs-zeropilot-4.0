#pragma once

#include "cmsis_os2.h"
#include <stdint.h>

#define MAX_PROFILED_TASKS 8

struct TaskProfile {
    const char* name;
    uint32_t delta;
};

void profilerRegister(const char* name, uint8_t* outId);
void profilerTick(uint8_t id);
void profilerGetAll(TaskProfile* out, uint8_t* count);
