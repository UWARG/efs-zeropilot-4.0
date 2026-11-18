#pragma once
#include "stm32h7xx.h"

#ifdef __cplusplus
#include <cstdint>
#endif

static constexpr uint32_t BUFFER_SIZE = 512;
static constexpr uint16_t RX_CAPACITY = BUFFER_SIZE - 1;
static constexpr uint16_t TX_BUFFER_SIZE = 512; // Must hold a full TM_MAX_TX_BYTES batch
