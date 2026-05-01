#pragma once 

#include <cstdint>

typedef struct {
    // Press and Temp data
    float pressureData;
    float temperatureData;
    float altitude;
} BaroData_t;

class IBarometer {
    protected:
        IBarometer() = default; 
    public:
        virtual ~IBarometer() = default;
        
        virtual bool readData(BaroData_t *data) = 0;
};