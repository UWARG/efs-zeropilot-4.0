#include "buzzer.hpp"

M10Buzzer::M10Buzzer(
	GPIO_TypeDef *buzzerPort,
	uint16_t buzzerPin
):
	buzzerPort(buzzerPort),
	buzzerPin(buzzerPin) {
}

void M10Buzzer::init() {
	buzzerOff();
}

void M10Buzzer::buzzerOn() {
	HAL_GPIO_WritePin(buzzerPort, buzzerPin, GPIO_PIN_SET);
}

void M10Buzzer::buzzerOff() {
	HAL_GPIO_WritePin(buzzerPort, buzzerPin, GPIO_PIN_RESET);
}
