#include "direct_mapping.hpp"

void DirectMapping::activateFlightMode() {
    // No activation tasks for DirectMapping
}

RCMotorControlMessage_t DirectMapping::runControl(RCMotorControlMessage_t controlInputs, const DroneState_t &droneState){
    return controlInputs;
}
