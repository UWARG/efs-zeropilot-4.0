#pragma once

#include "param_setup.hpp"

class TelemetryManager;

class TMParamSetup : public IParamSetup {
   public:
    explicit TMParamSetup(TelemetryManager* tm);
    void loadAllParams() override;
    void bindAllParamCallbacks() override;

   private:
    TelemetryManager* tm;
};
