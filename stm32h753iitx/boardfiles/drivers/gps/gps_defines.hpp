#pragma once

#include <cstdint>

static constexpr uint8_t MAX_NMEA_DATA_LENGTH_PER_LINE = 82;
static constexpr uint8_t NUM_NMEA_DATA_LINES = 8;
static constexpr uint16_t MAX_NMEA_DATA_LENGTH = MAX_NMEA_DATA_LENGTH_PER_LINE * NUM_NMEA_DATA_LINES;

static constexpr uint32_t DECIMAL_PRECISION = 1e6;
