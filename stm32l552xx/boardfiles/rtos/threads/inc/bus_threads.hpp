#pragma once

#include "cmsis_os2.h"
#include <cstdint>

static constexpr uint16_t BUS_UPDATE_LOOP_DELAY_MS = 50; // 20 Hz

extern osThreadId_t busMainHandle;
extern volatile uint32_t busMainLoopCount;
extern volatile uint32_t busFreeHeapBeforeCreate;
extern volatile uint32_t busFreeHeapAfterCreate;

void busInitThreads();
