#pragma once

class ISafetySwitch {
    protected:
    ISafetySwitch() = default;

    public:
    virtual ~ISafetySwitch() = default;

    virtual bool isPressed() = 0; 
};