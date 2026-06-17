#include "direct_mapping.hpp"

ZP_ERROR_e DirectMapping::activateFlightMode() {
    // No activation tasks for DirectMapping
}


ZP_ERROR_e DirectMapping::runControl(RCMotorControlMessage_t &motorOutputs, RCMotorControlMessage_t controlInput, const DroneState_t &droneState){
    // Copy control inputs directly to motor outputs
    motorOutputs = controlInput;

    return ZP_ERROR_OK;
}
