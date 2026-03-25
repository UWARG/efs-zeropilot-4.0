#pragma once

class IAirspeed {
protected:
    IAirspeed() = default;
public:
    virtual ~IAirspeed() = default;

    virtual bool getAirspeedData(double* data_out) = 0;
};
