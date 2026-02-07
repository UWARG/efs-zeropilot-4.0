#pragma once
#include "iwdg_iface.hpp"

class SIL_IWDG : public IIndependentWatchdog {
public:
    bool refreshWatchdog() override { return true; }
};
