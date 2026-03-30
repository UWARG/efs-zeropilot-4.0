#pragma once

class ILed {
protected:
    ILed() = default;

public: 
    virtual ~ILed() = default;

    virtual void ledOn() = 0;
    virtual void ledOff() = 0;
};