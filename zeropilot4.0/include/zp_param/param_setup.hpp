#pragma once

#include "zp_error.h"

class IParamSetup {
   public:
    virtual ~IParamSetup() = default;
    virtual ZP_Error loadAllParams() = 0;
    virtual ZP_Error bindAllParamCallbacks() = 0;
};
