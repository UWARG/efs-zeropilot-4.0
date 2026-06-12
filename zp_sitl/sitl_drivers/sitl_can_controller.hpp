#pragma once
#include "can_controller_iface.hpp"

class SITL_CANController : public ICANController {
public:
    bool routineTasks() override { return true; }
};
