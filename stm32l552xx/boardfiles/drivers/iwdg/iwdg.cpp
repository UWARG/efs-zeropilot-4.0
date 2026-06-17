#include "iwdg.hpp"

IndependentWatchdog::IndependentWatchdog(IWDG_HandleTypeDef *hiwdg) : watchdog_(hiwdg) {
    // empty
}

ZP_ERROR_e IndependentWatchdog::refreshWatchdog() {
    if (this->watchdog_ == nullptr) {
        return ZP_ERROR_NULLPTR;
    }

    if (HAL_IWDG_Refresh(this->watchdog_) == HAL_OK) return ZP_ERROR_OK;
}
