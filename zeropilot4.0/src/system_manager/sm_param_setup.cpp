#include "sm_param_setup.hpp"
#include "system_manager.hpp"
#include "zp_params.hpp"
#include "flightmode.hpp"

SMParamSetup::SMParamSetup(SystemManager* sm) : sm(sm) {}

ZP_ERROR_e SMParamSetup::loadAllParams() {
    ZP_ERROR_e result = ZP_ERROR_OK;
    static constexpr ZP_PARAM_ID FLTMODE_PARAMS[SM_FLIGHTMODE_COUNT] = {
        ZP_PARAM_ID::FLTMODE1, ZP_PARAM_ID::FLTMODE2, ZP_PARAM_ID::FLTMODE3,
        ZP_PARAM_ID::FLTMODE4, ZP_PARAM_ID::FLTMODE5, ZP_PARAM_ID::FLTMODE6
    };
    for (uint8_t i = 0; i < SM_FLIGHTMODE_COUNT; i++) {
        float val = 0.0f;
        result |= ZP_PARAM::get(FLTMODE_PARAMS[i], val);
        if (result == ZP_ERROR_OK) {
            sm->flightModes[i] = static_cast<PlaneFlightMode_e>(
                static_cast<uint32_t>(val));
        }
    }
    return result;
}

ZP_ERROR_e SMParamSetup::bindAllParamCallbacks() {
    ZP_ERROR_e result = ZP_ERROR_OK;
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::FLTMODE1, sm, updateFltMode1);
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::FLTMODE2, sm, updateFltMode2);
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::FLTMODE3, sm, updateFltMode3);
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::FLTMODE4, sm, updateFltMode4);
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::FLTMODE5, sm, updateFltMode5);
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::FLTMODE6, sm, updateFltMode6);
    return result;
}

ZP_ERROR_e SMParamSetup::setFltMode(SystemManager* ctx, uint8_t idx, float val) {
    uint32_t mode = static_cast<uint32_t>(val);
    if (!isValidPlaneFlightMode(mode)) return ZP_ERROR_INVALID_PARAM;
    ctx->flightModes[idx] = static_cast<PlaneFlightMode_e>(mode);
    return ZP_ERROR_OK;
}

// Flightmode param callbacks
ZP_ERROR_e SMParamSetup::updateFltMode1(SystemManager* ctx, float val) { return setFltMode(ctx, 0, val); }
ZP_ERROR_e SMParamSetup::updateFltMode2(SystemManager* ctx, float val) { return setFltMode(ctx, 1, val); }
ZP_ERROR_e SMParamSetup::updateFltMode3(SystemManager* ctx, float val) { return setFltMode(ctx, 2, val); }
ZP_ERROR_e SMParamSetup::updateFltMode4(SystemManager* ctx, float val) { return setFltMode(ctx, 3, val); }
ZP_ERROR_e SMParamSetup::updateFltMode5(SystemManager* ctx, float val) { return setFltMode(ctx, 4, val); }
ZP_ERROR_e SMParamSetup::updateFltMode6(SystemManager* ctx, float val) { return setFltMode(ctx, 5, val); }