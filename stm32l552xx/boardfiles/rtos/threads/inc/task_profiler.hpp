#pragma once

#include <stdint.h>

#define MAX_PROFILED_TASKS 8

struct TaskProfile {
    const char* name;
    uint32_t delta;
};

void profilerRegister(const char* name, uint8_t* outId);
void profilerBegin(uint8_t id);
void profilerEnd(uint8_t id);
void profilerGetAll(TaskProfile* out, uint8_t* count);
