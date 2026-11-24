#pragma once
#include "zp_error.h"


//airspeed sensor data 
typedef struct {
    float speed; // The detected speed
    float variance; // The detected variance

    bool isValid; //tells the caller whether it can use the sample
    bool isNew; //whether the driver has saved a sample since the previous read
} AirspeedData_t;


class IAirspeed{
    private: 

    public: 
        virtual ~IAirspeed() = default; //destructor operation
        virtual ZP_Error readAirspeedData(AirspeedData_t &data) = 0;
};