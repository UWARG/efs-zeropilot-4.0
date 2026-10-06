#pragma once

class ISafetySwitch {
    protected:
        ISafetySwitch() = default;

    public:
        virtual ~ISafetySwitch() = default;

        // check if safety switch is pressed
        virtual bool isSafetySwitchPressed() = 0;

        // set/get the state of the safety switch LED
        virtual void setSafetySwitchLEDState(bool state) = 0;
        virtual bool getSafetySwitchLEDState() = 0;
};
