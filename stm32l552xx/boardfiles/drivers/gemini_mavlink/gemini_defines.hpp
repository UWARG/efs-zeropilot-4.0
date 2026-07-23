#pragma once

#include "stm32l5xx.h"
#include <cstdint>
#include "mavlink.h"

static constexpr uint16_t MAVLINK_MAX_PACKET_SIZE = 280; 




//---CRSF Defines----
// Note: taken directly from rc_defines.hpp

static constexpr uint8_t  CRSF_BYTE_COUNT     = 64;
static constexpr uint8_t  CRSF_SYNC_BYTE      = 0xC8;     // CRSF SYNC for most packets
static constexpr uint8_t  CRSF_SYNC_ALT       = 0xEE;     // EdgeTX “outgoing” SYNC

static constexpr uint16_t CRSF_PULSE_MIN = 172; //represents 988us (microseconds)
static constexpr uint16_t CRSF_PULSE_MAX = 1811; //represents 2012us (microseconds)
static constexpr uint16_t CRSF_PULSE_RANGE = (CRSF_PULSE_MAX - CRSF_PULSE_MIN);

static constexpr uint16_t CRSF_AUX_MIN = 191; //represents 1000us (microseconds)
static constexpr uint16_t CRSF_AUX_MAX = 1792; //represents 2000us (microseconds)
static constexpr uint16_t CRSF_AUX_RANGE = (CRSF_AUX_MAX - CRSF_AUX_MIN);

//Frame types (can add more when implementing other frame types. Only adding RC channel frametype for now)
static constexpr uint8_t  CRSF_FRAMETYPE_RC_CHANNELS_PACKED = 0x16;
