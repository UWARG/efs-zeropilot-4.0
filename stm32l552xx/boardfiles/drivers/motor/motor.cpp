#include "motor.hpp"

MotorControl::MotorControl(TIM_HandleTypeDef *timer, uint32_t timerChannel, uint32_t minDutyCycle, uint32_t maxDutyCycle, uint8_t servoIdx) : 
    timer(timer), 
    timerChannel(timerChannel), 
    minCCR(minDutyCycle / 100.0 * timer->Init.Period), 
    maxCCR(maxDutyCycle / 100.0 * timer->Init.Period),
    servoIdx(servoIdx) {
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
