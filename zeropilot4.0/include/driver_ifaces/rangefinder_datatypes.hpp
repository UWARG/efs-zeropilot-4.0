#pragma once

#include <cstdint>

typedef struct {
    uint16_t distance; // cm
    uint16_t signalStrength; // Higher is better
    uint16_t temp; // Chip temperature in celsius 
    bool isValid; // True if reading is valid, false otherwise
} RangefinderData_t;
