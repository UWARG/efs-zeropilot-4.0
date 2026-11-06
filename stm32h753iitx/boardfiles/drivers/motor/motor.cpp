#include "motor.hpp"

MotorControl::MotorControl(TIM_HandleTypeDef *timer, uint32_t timerChannel, uint32_t minDutyCycle, uint32_t maxDutyCycle, uint8_t servoIdx) : 
    timer(timer), 
    timerChannel(timerChannel), 
    minCCR(minDutyCycle / 100.0 * timer->Init.Period), 
    maxCCR(maxDutyCycle / 100.0 * timer->Init.Period),
    servoIdx(servoIdx) {}

ZP_Error MotorControl::set(uint32_t percent) {
    percent = percent > 100 ? 100 : percent;

    uint32_t ticks = 0;
    ticks = ((percent / 100.0) * (maxCCR - minCCR)) + minCCR;

    __HAL_TIM_SET_COMPARE(timer, timerChannel, ticks);
    return ZP_ERROR_OK;
}

ZP_Error MotorControl::init() {
    if (timer == nullptr) {
        return ZP_ERROR_NULLPTR;
    }

    __HAL_TIM_SET_COMPARE(timer, timerChannel, minCCR);

    if (HAL_TIM_PWM_Start(timer, timerChannel) != HAL_OK) {
        return ZP_ERROR_EXT_API | ZP_ERROR_FAIL;
    }
    return ZP_ERROR_OK;
}

void MotorControl::enableServo(GPIO_TypeDef* enGpioBase, uint16_t enGpioNum) {
    HAL_GPIO_WritePin(enGpioBase, enGpioNum, GPIO_PIN_SET);
}

ZP_Error MotorControl::enableServoSwitch(GPIO_TypeDef* csGpioBase, uint16_t csGpioNum, SPI_HandleTypeDef *hspi) {
    uint8_t rx[2];
    static constexpr uint8_t tx[2] = {0xFF, 0xAC};

    HAL_GPIO_WritePin(csGpioBase, csGpioNum, GPIO_PIN_RESET);
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(hspi, tx, rx, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(csGpioBase, csGpioNum, GPIO_PIN_SET);

    if (status == HAL_OK) {
        return ZP_ERROR_OK;
    } else {
        return ZP_ERROR_FAIL;
    }
}
