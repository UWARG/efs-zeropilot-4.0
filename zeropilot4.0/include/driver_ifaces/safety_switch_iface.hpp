#pragma once

class ISafetySwitch {
    protected:
    ISafetySwitch() = default;

    public:
    virtual ~ISafetySwitch() = default;

    virtual bool isOn() = 0;
};
