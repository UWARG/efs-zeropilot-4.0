#include "direct_mapping.hpp"

ZP_ERROR_e DirectMapping::runControl(RCMotorControlMessage_t *motorOutputs, RCMotorControlMessage_t controlInputs){
    // Check for null pointer
    if (motorOutputs == nullptr) {
        return ZP_ERROR_NULLPTR;
    }

    // Copy control inputs directly to motor outputs
    *motorOutputs = controlInputs;

    return ZP_ERROR_OK;
}
