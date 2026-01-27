#include "iwdg.hpp"

IndependentWatchdog::IndependentWatchdog(IWDG_HandleTypeDef *hiwdg) : watchdog_(hiwdg) {
    // empty
}

bool IndependentWatchdog::refreshWatchdog() {
    if (this->watchdog_ == nullptr) {
        return false;
    }

    return (HAL_IWDG_Refresh(this->watchdog_) == HAL_OK);
}
