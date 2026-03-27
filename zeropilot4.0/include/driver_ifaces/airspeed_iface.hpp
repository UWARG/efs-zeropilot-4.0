#pragma once

typedef struct {
    double raw_press_ = 0;
    double raw_temp_ = 0;
    double processed_temp_ = 0;
    double processed_press_ = 0;
    double airspeed_ = 0;
} AirspeedData_t ;

enum class Status : uint8_t {
    Normal  = 0b00,
    Command = 0b01,
    Stale   = 0b10,
    Fault   = 0b11
};

class IAirspeed {
protected:
    IAirspeed() = default;
public:
    virtual ~IAirspeed() = default;
    
    virtual bool getAirspeedData(AirspeedData_t* data) = 0;
};
