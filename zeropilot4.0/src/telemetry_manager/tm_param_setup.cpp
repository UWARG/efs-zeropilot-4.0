#include "tm_param_setup.hpp"
#include "telemetry_manager.hpp"

TMParamSetup::TMParamSetup(TelemetryManager* tm) : tm(tm) {}

ZP_Error TMParamSetup::loadAllParams() {
    return ZP_ERROR_OK;
}

ZP_Error TMParamSetup::bindAllParamCallbacks() {
    return ZP_ERROR_OK;
}
