#include "m10_accessory.hpp"

M10Accessory::M10Accessory(
	GPIO_TypeDef *safetySwitchPort,
	uint16_t safetySwitchPin,
	GPIO_TypeDef *buzzerPort,
	uint16_t buzzerPin
) :
	safetySwitchPort(safetySwitchPort),
	safetySwitchPin(safetySwitchPin),
	buzzerPort(buzzerPort),
	buzzerPin(buzzerPin) {
}

void M10Accessory::init() {
	buzzerOff();
}

bool M10Accessory::readSafetySwitch() {
	return HAL_GPIO_ReadPin(safetySwitchPort, safetySwitchPin) == GPIO_PIN_SET;
}

void M10Accessory::buzzerOn() {
	HAL_GPIO_WritePin(buzzerPort, buzzerPin, GPIO_PIN_SET);
}

void M10Accessory::buzzerOff() {
	HAL_GPIO_WritePin(buzzerPort, buzzerPin, GPIO_PIN_RESET);
}
