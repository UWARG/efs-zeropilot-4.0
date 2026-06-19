#pragma once

#include <cstdint>
#include "zp_error.h"
#include "param_setup.hpp"

class SystemManager;

class SMParamSetup : public IParamSetup {
    public:
        explicit SMParamSetup(SystemManager* sm);
        ZP_ERROR_e loadAllParams() override;
        ZP_ERROR_e bindAllParamCallbacks() override;

    private:
        SystemManager* sm;

        // Flightmode helper
        static ZP_ERROR_e setFltMode(SystemManager* ctx, uint8_t idx, float val);
        // Flightmode param callbacks
        static ZP_ERROR_e updateFltMode1(SystemManager* ctx, float val);
        static ZP_ERROR_e updateFltMode2(SystemManager* ctx, float val);
        static ZP_ERROR_e updateFltMode3(SystemManager* ctx, float val);
        static ZP_ERROR_e updateFltMode4(SystemManager* ctx, float val);
        static ZP_ERROR_e updateFltMode5(SystemManager* ctx, float val);
        static ZP_ERROR_e updateFltMode6(SystemManager* ctx, float val);
};
