#include "safety_switch.hpp"

SafetySwitch::SafetySwitch(GPIO_TypeDef* switchPort, uint16_t switchPin, GPIO_TypeDef* ledPort, uint16_t ledPin) :
    switchPort(switchPort),
    switchPin(switchPin),
    ledPort(ledPort),
    ledPin(ledPin),
    ledState(false) {

    HAL_GPIO_WritePin(ledPort, ledPin, GPIO_PIN_SET);  // Initialize LED to OFF state
}

SafetySwitch::~SafetySwitch() {}

bool SafetySwitch::isSafetySwitchPressed() {
    return (HAL_GPIO_ReadPin(switchPort, switchPin) == GPIO_PIN_SET);  // Active high switch
}

void SafetySwitch::setSafetySwitchLEDState(bool state) {
    ledState = state;
    HAL_GPIO_WritePin(ledPort, ledPin, state ? GPIO_PIN_RESET : GPIO_PIN_SET); // Active low LED
}

bool SafetySwitch::getSafetySwitchLEDState() {
    return ledState;
}
