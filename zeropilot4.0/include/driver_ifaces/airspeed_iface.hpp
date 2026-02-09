#pragma once

class AirspeedIFace
{
protected:
    AirspeedIFace() = default;
public:
    virtual ~AirspeedIFace() = default;
    
    virtual void getAirspeedData() = 0;
};