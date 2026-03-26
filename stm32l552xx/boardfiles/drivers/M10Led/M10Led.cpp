#include "M10Led.hpp"

M10Led::M10Led(
	GPIO_TypeDef *ledPort,
	uint16_t ledPin
):
	ledPort(ledPort),
	ledPin(ledPin) {
}

void M10Led::init() {
	ledOff();
}

void M10Led::ledOn() {
	HAL_GPIO_WritePin(ledPort, ledPin, GPIO_PIN_SET);
}

void M10Led::ledOff() {
	HAL_GPIO_WritePin(ledPort, ledPin, GPIO_PIN_RESET);
}
