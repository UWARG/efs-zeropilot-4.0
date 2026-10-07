#include "direct_mapping.hpp"

ZP_Error DirectMapping::activateFlightMode() {
    // No activation tasks for DirectMapping
    return ZP_ERROR_OK;
}


ZP_Error DirectMapping::runControl(RCMotorControlMessage_t &motorOutputs, RCMotorControlMessage_t controlInput, const DroneState_t &droneState){
    // Copy control inputs directly to motor outputs
    motorOutputs = controlInput;

    return ZP_ERROR_OK;
}
