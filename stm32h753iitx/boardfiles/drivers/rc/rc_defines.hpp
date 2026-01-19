#pragma once
#include <cstdint>

static constexpr uint8_t SBUS_PACKET_SIZE = 25;
static constexpr uint8_t SBUS_CHANNEL_COUNT = 16;
static constexpr uint8_t SBUS_MAX_BTYES_PER_CHANNEL = 3;

/* define range mapping here, -+100% -> 1000..2000 */
static constexpr uint16_t SBUS_RANGE_MIN = 192;
static constexpr uint16_t SBUS_RANGE_MAX = 1792;
static constexpr uint16_t SBUS_RANGE_RANGE = (SBUS_RANGE_MAX - SBUS_RANGE_MIN);

static constexpr uint8_t HEADER_ = 0x0F;
static constexpr uint8_t FOOTER_ = 0x00;

//---CRSF Defines----

static constexpr uint8_t  CRSF_PACKET_SIZE     = 64;
static constexpr uint8_t  CRSF_SYNC_BYTE      = 0xC8;     // CRSF SYNC for most packets
static constexpr uint8_t  CRSF_SYNC_ALT       = 0xEE;     // EdgeTX “outgoing” SYNC

static constexpr uint16_t CRSF_PULSE_MIN = 172; //represents 988us (microseconds)
static constexpr uint16_t CRSF_PULSE_MAX = 1811; //represents 2012us (microseconds)
static constexpr uint16_t CRSF_PULSE_RANGE = (CRSF_PULSE_MAX - CRSF_PULSE_MIN);

//Frame types (can add more when implementing other frame types. Only adding RC channel frametype for now)
static constexpr uint8_t  CRSF_FRAMETYPE_RC_CHANNELS_PACKED = 0x16;
