#include "iwdg.hpp"

IndependentWatchdog::IndependentWatchdog(IWDG_HandleTypeDef *hiwdg) : watchdog_(hiwdg) {
    // empty
}

ZP_Error IndependentWatchdog::refreshWatchdog() {
    if (this->watchdog_ == nullptr) {
        return ZP_ERROR_NULLPTR;
    }

    if (HAL_IWDG_Refresh(this->watchdog_) == HAL_OK) {
        return ZP_ERROR_OK;
    } else {
        return ZP_ERROR_EXT_API | ZP_ERROR_FAIL;
    };
}
