#pragma once

#include <cstdint>

#include "param_setup.hpp"

class SystemManager;

class SMParamSetup : public IParamSetup {
   public:
    explicit SMParamSetup(SystemManager* sm);
    void loadAllParams() override;
    void bindAllParamCallbacks() override;

   private:
    SystemManager* sm;

    // Flightmode helper
    static bool setFltMode(SystemManager* ctx, uint8_t idx, float val);

    // Flightmode param callbacks
    static bool updateFltMode1(SystemManager* ctx, float val);
    static bool updateFltMode2(SystemManager* ctx, float val);
    static bool updateFltMode3(SystemManager* ctx, float val);
    static bool updateFltMode4(SystemManager* ctx, float val);
    static bool updateFltMode5(SystemManager* ctx, float val);
    static bool updateFltMode6(SystemManager* ctx, float val);
};
