#pragma once

class airspeed_iface
{
protected:
    airspeed_iface() = default;
public:
    virtual ~airspeed_iface() = default;
    
    virtual void getData() = 0;
    virtual void calculateAirspeed() = 0;
};