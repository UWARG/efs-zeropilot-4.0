#pragma once

#include "iwdg_iface.hpp"
#include "stm32l5xx_hal.h"
#include "error.h"

class IndependentWatchdog : public IIndependentWatchdog{
    private:
        IWDG_HandleTypeDef * const watchdog_;

    public:
        IndependentWatchdog(IWDG_HandleTypeDef *hiwdg);

        /**
         * @brief reset watchdog timer
         */
        ZP_ERROR_e refreshWatchdog() override;
};
