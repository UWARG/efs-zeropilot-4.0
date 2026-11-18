#pragma once

#include <cstdint>
#include "zp_error.h"
#include "param_setup.hpp"

class SystemManager;

class SMParamSetup : public IParamSetup {
    public:
        explicit SMParamSetup(SystemManager* sm);
        ZP_Error loadAllParams() override;
        ZP_Error bindAllParamCallbacks() override;

    private:
        SystemManager* sm;

    // Flightmode helper
    static ZP_Error setFltMode(SystemManager* ctx, uint8_t idx, float val);
    static ZP_Error setRCReversed(SystemManager* ctx, uint8_t idx, float val);

    // Flightmode param callbacks
    static ZP_Error updateFltMode1(SystemManager* ctx, float val);
    static ZP_Error updateFltMode2(SystemManager* ctx, float val);
    static ZP_Error updateFltMode3(SystemManager* ctx, float val);
    static ZP_Error updateFltMode4(SystemManager* ctx, float val);
    static ZP_Error updateFltMode5(SystemManager* ctx, float val);
    static ZP_Error updateFltMode6(SystemManager* ctx, float val);

    // Channel reverse callbacks
    static ZP_Error setRC1Reversed(SystemManager* ctx, float val);
    static ZP_Error setRC2Reversed(SystemManager* ctx, float val);
    static ZP_Error setRC3Reversed(SystemManager* ctx, float val);
    static ZP_Error setRC4Reversed(SystemManager* ctx, float val);
};
