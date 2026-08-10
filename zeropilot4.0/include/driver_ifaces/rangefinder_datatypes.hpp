#pragma once

#include <cstdint>

typedef struct {
    float distance; // m
    uint16_t signalStrength; // Higher is better
    int16_t temp; // Chip temperature in celsius 
    bool isValid; // True if data is valid, false otherwise
    bool isNew; // True if the data is new since last readData()
} RangefinderData_t;
