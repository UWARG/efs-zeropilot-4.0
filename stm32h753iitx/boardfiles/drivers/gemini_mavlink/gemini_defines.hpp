#pragma once

#include "stm32h7xx_hal.h"
#include <cstdint>
#include "mavlink.h"

static constexpr uint16_t MAVLINK_MAX_PACKET_SIZE = 280; 