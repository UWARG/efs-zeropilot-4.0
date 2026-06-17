#include "direct_mapping.hpp"

ZP_ERROR_e DirectMapping::activateFlightMode() {
    // No activation tasks for DirectMapping
}


ZP_ERROR_e DirectMapping::runControl(RCMotorControlMessage_t &motorOutputs, const DroneState_t &droneState, RCMotorControlMessage_t controlInputs){
    // Copy control inputs directly to motor outputs
    motorOutputs = controlInputs;

    return ZP_ERROR_OK;
}
