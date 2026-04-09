#pragma once 

#include <cstdint>

typedef struct {
    // Press and Temp data
    uint8_t Press_Temp_Data[6];
    uint8_t FIFO_REGISTER; 
} BaroData_t;

class IBarometer {
    protected:
        IBarometer() = default; 
    public:
        virtual ~IBarometer() = default;
        
        virtual bool readData(BaroData_ *data) = 0;
}