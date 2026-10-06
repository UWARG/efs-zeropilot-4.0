#pragma once

#include "cmsis_os2.h"

static constexpr uint16_t BUS_UPDATE_LOOP_DELAY_MS = 100; // 10 Hz

void busInitThreads();
