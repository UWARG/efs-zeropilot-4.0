#pragma once

class AirspeedIFace
{
protected:
    AirspeedIFace() = default;
public:
    virtual ~AirspeedIFace() = default;
    
    virtual bool getAirspeedData(double* data_out) = 0;
};
