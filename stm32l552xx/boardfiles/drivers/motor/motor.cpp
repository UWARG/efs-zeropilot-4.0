#include "motor.hpp"

MotorControl::MotorControl(TIM_HandleTypeDef *timer, uint32_t timerChannel, uint32_t minDutyCycle, uint32_t maxDutyCycle) : 
    timer(timer), 
    timerChannel(timerChannel), 
    minCCR(minDutyCycle / 100.0 * timer->Init.Period), 
    maxCCR(maxDutyCycle / 100.0 * timer->Init.Period) {
    // blank
}

ZP_ERROR_e MotorControl::set(uint32_t percent) {
    if (timer == NULL) {
        return ZP_ERROR_NULLPTR;
    }

    percent = percent > 100 ? 100 : percent;
    uint32_t ticks = ((percent / 100.0) * (maxCCR - minCCR)) + minCCR;
    __HAL_TIM_SET_COMPARE(timer, timerChannel, ticks);

    return ZP_ERROR_OK;
}

ZP_ERROR_e MotorControl::init() {
    if (timer == NULL) {
        return ZP_ERROR_NULLPTR;
    }

    __HAL_TIM_SET_COMPARE(timer, timerChannel, minCCR);
    HAL_StatusTypeDef status = HAL_TIM_PWM_Start(timer, timerChannel);
    if (status != HAL_OK) {
        return ZP_ERROR_FAIL;
    }

    return ZP_ERROR_OK;
}
