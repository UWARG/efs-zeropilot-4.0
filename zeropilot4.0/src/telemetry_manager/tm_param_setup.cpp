#include "tm_param_setup.hpp"
#include "telemetry_manager.hpp"

TMParamSetup::TMParamSetup(TelemetryManager* tm) : tm(tm) {}

ZP_ERROR_e TMParamSetup::loadAllParams() {
    return ZP_ERROR_OK;
}

ZP_ERROR_e TMParamSetup::bindAllParamCallbacks() {
    return ZP_ERROR_OK;
}
