#pragma once

#include "safety_switch_iface.hpp"
#include "stm32l5xx_hal.h"

class SafetySwitch : public ISafetySwitch {
    public:
        SafetySwitch(GPIO_TypeDef* switchPort, uint16_t switchPin, GPIO_TypeDef* ledPort, uint16_t ledPin);
        ~SafetySwitch() override;

        bool isSafetySwitchPressed() override;
        void setSafetySwitchLEDState(bool state) override;
        bool getSafetySwitchLEDState() override;

    private:
        GPIO_TypeDef* switchPort;
        uint16_t switchPin;
        GPIO_TypeDef* ledPort;
        uint16_t ledPin;
        bool ledState;
};
