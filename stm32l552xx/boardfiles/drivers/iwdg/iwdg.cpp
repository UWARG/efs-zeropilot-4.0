#include "iwdg.hpp"

IndependentWatchdog::IndependentWatchdog(IWDG_HandleTypeDef *hiwdg) : watchdog_(hiwdg) {
    // empty
}

ZP_ERROR_e IndependentWatchdog::refreshWatchdog() {
    if (this->watchdog_ == nullptr) {
        return ZP_ERROR_NULLPTR;
    }

    HAL_StatusTypeDef status = HAL_IWDG_Refresh(this->watchdog_);
    if (status != HAL_OK) {
        return ZP_ERROR_FAIL;
    }

    return ZP_ERROR_OK;
}
