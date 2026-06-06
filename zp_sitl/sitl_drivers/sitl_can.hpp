#pragma once
#include "can_iface.hpp"

class SITL_CAN : public ICAN {
public:
    bool routineTasks() override { return true; }
};
