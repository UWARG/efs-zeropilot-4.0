#pragma once

class IM10Accessory {
    protected:
        IM10Accessory() = default;

    public:
        virtual ~IM10Accessory() = default;

        virtual bool readSafetySwitch() = 0;

        virtual void buzzerOn() = 0;

        virtual void buzzerOff() = 0;
};