#pragma once
#include "zp_error.h"
#include "param_setup.hpp"

class TelemetryManager;

class TMParamSetup : public IParamSetup {
   public:
    explicit TMParamSetup(TelemetryManager* tm);
    ZP_ERROR_e loadAllParams() override;
    ZP_ERROR_e bindAllParamCallbacks() override;

   private:
    TelemetryManager* tm;
};
