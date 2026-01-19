#include "motor.hpp"

MotorControl::MotorControl(TIM_HandleTypeDef *timer, uint32_t timerChannel, uint32_t minDutyCycle, uint32_t maxDutyCycle) : 
    timer(timer), 
    timerChannel(timerChannel), 
    minCCR(minDutyCycle / 100.0 * timer->Init.Period), 
    maxCCR(maxDutyCycle / 100.0 * timer->Init.Period) {
    // blank
}

void MotorControl::set(uint32_t percent) {
    percent = percent > 100 ? 100 : percent;
    uint32_t ticks = ((percent / 100.0) * (maxCCR - minCCR)) + minCCR;
    __HAL_TIM_SET_COMPARE(timer, timerChannel, ticks);
}

void MotorControl::init() {
    __HAL_TIM_SET_COMPARE(timer, timerChannel, minCCR);
    HAL_TIM_PWM_Start(timer, timerChannel);
}

void MotorControl::enableServo(GPIO_TypeDef* enGpioBase, uint16_t enGpioNum) {
    HAL_GPIO_WritePin(enGpioBase, enGpioNum, GPIO_PIN_SET);
}

void MotorControl::enableServoSwitch(GPIO_TypeDef* csGpioBase, uint16_t csGpioNum, SPI_HandleTypeDef *hspi) {
    uint8_t rx[2], tx[2];
    tx[0] = 0xFF;
    tx[1] = 0xAC;

    HAL_GPIO_WritePin(csGpioBase, csGpioNum, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(hspi, tx, rx, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(csGpioBase, csGpioNum, GPIO_PIN_SET);
}