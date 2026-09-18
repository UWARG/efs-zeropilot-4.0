#pragma once 

#include <cstdint>
#include "zp_error.h"

// Pressure and Temperature data
typedef struct {
    float pressureKPa; // kPa
    float temperatureC; // Celsius
    float altitude; // meters
} BaroData_t;

class IBarometer {
    protected:
        IBarometer() = default; 
    public:
        virtual ~IBarometer() = default;
        
        virtual ZP_Error readData(BaroData_t &data) = 0;
};
