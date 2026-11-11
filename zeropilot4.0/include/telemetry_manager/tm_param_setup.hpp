#pragma once
#include "zp_error.h"
#include "param_setup.hpp"

class TelemetryManager;

class TMParamSetup : public IParamSetup {
   public:
    explicit TMParamSetup(TelemetryManager* tm);
    ZP_Error loadAllParams() override;
    ZP_Error bindAllParamCallbacks() override;

   private:
    TelemetryManager* tm;
};
