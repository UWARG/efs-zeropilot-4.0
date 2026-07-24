#pragma once 

#include <cstdint>

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
        
        virtual bool readData(BaroData_t &data) = 0;
};
