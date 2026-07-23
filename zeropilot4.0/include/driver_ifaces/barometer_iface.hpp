#pragma once 

#include <cstdint>

typedef struct {
    // Press and Temp data
    float pressureKPa; // kPa
    float temperatureC; // Celsius
    float altitude; // M
} BaroData_t;

class IBarometer {
    protected:
        IBarometer() = default; 
    public:
        virtual ~IBarometer() = default;
        
        virtual bool readData(BaroData_t &data) = 0;
};