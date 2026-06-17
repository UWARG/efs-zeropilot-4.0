#pragma once

class IParamSetup {
   public:
    virtual ~IParamSetup() = default;
    virtual ZP_ERROR_e loadAllParams() = 0;
    virtual ZP_ERROR_e bindAllParamCallbacks() = 0;
};
