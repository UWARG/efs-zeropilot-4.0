#pragma once

#include "stm32l5xx_hal.h"
#include "m10_accessory_iface.hpp"

class M10Accessory : public IM10Accessory {
public:
	M10Accessory(GPIO_TypeDef *safetySwitchPort, uint16_t safetySwitchPin, GPIO_TypeDef *buzzerPort, uint16_t buzzerPin);

	bool readSafetySwitch() override;
	void buzzerOn() override;
	void buzzerOff() override;
    
	void init();

private:
	GPIO_TypeDef * const safetySwitchPort;
	const uint16_t safetySwitchPin;

	GPIO_TypeDef * const buzzerPort;
	const uint16_t buzzerPin;
};