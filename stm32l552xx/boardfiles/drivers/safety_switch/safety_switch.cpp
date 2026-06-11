#include "safety_switch.hpp"

M10SafetySwitch::M10SafetySwitch(
    GPIO_TypeDef *safetySwitchPort,
    uint16_t safetySwitchPin
) :
    safetySwitchPort(safetySwitchPort),
    safetySwitchPin(safetySwitchPin) {
}

bool M10SafetySwitch::isOn() {
    const bool rawPressed = HAL_GPIO_ReadPin(safetySwitchPort, safetySwitchPin) == GPIO_PIN_RESET;

    if (!rawStateInitialized) {
        safetySwitchOn = false;
        lastRawPressed = rawPressed;
        rawStateInitialized = true;
        toggleArmed = !rawPressed;
        return safetySwitchOn;
    }

    if (!toggleArmed) {
        if (!rawPressed) {
            toggleArmed = true;
        }

        lastRawPressed = rawPressed;
        return false;
    }

    if (rawPressed && !lastRawPressed) {
        safetySwitchOn = !safetySwitchOn;
    }

    lastRawPressed = rawPressed;
    return safetySwitchOn;
}
