#pragma once

#include "stm32l5xx_hal.h"
#include "led_iface.hpp"

class M10Led : public ILed {
public:
	M10Led(GPIO_TypeDef *ledPort, uint16_t ledPin);

	void ledOn() override;
	void ledOff() override;
    
	void init();

private:
	GPIO_TypeDef * const ledPort;
	const uint16_t ledPin;
};