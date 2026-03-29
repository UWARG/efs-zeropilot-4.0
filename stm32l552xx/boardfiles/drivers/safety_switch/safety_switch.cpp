#include "safety_switch.hpp"

M10SafetySwitch::M10SafetySwitch(
    GPIO_TypeDef *safetySwitchPort,
    uint16_t safetySwitchPin
) :
    safetySwitchPort(safetySwitchPort),
    safetySwitchPin(safetySwitchPin) {
}

bool M10SafetySwitch::isPressed() {
    return HAL_GPIO_ReadPin(safetySwitchPort, safetySwitchPin) == GPIO_PIN_RESET;
}
