#pragma once

#include "stm32l5xx.h"
#include <cstdint>
#include "mavlink.h"

static constexpr uint16_t MAVLINK_MAX_PACKET_SIZE = 280; 
static constexpr uint8_t MAVLINK_CHANNEL = MAVLINK_COMM_0;