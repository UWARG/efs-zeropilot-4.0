#pragma once

#include "stm32l5xx_hal.h"
#include "buzzer_iface.hpp"

class M10Buzzer : public IBuzzer {
public:
	M10Buzzer(GPIO_TypeDef *buzzerPort, uint16_t buzzerPin);

	void buzzerOn() override;
	void buzzerOff() override;
    
	void init();

private:
	GPIO_TypeDef * const buzzerPort;
	const uint16_t buzzerPin;
};