#pragma once

#include "stm32l5xx_hal.h"
#include "safety_switch_iface.hpp"

class M10SafetySwitch : public ISafetySwitch {
public:
	M10SafetySwitch(GPIO_TypeDef *safetySwitchPort, uint16_t safetySwitchPin);

    bool isOn() override;
    

private:
	GPIO_TypeDef * const safetySwitchPort;
	const uint16_t safetySwitchPin;
	bool safetySwitchOn = false;
	bool lastRawPressed = false;
	bool rawStateInitialized = false;
	bool toggleArmed = false;
};
