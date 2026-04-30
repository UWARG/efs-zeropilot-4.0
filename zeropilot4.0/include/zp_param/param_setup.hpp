#pragma once

class IParamSetup {
   public:
    virtual ~IParamSetup() = default;
    virtual void loadAllParams() = 0;
    virtual void bindAllParamCallbacks() = 0;
};
