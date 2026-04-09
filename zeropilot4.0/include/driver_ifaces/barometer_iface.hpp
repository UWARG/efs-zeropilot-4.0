#pragma once 

#include <cstdint>

typedef struct {
    // Press and Temp data
    float pressure_data;
    float temperature_data;
    float altitude;
} BaroData_t;

class IBarometer {
    protected:
        IBarometer() = default; 
    public:
        virtual ~IBarometer() = default;
        
        virtual bool readData(BaroData_t *data) = 0;
}