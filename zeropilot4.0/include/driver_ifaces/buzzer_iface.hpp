#pragma once

class IBuzzer {
protected: 
    IBuzzer() = default;

public:
    virtual ~IBuzzer() = default;

    virtual void buzzerOn() = 0;
    virtual void buzzerOff() = 0;
};