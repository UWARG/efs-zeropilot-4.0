#include "sm_param_setup.hpp"
#include "system_manager.hpp"
#include "zp_params.hpp"
#include "flightmode.hpp"

SMParamSetup::SMParamSetup(SystemManager* sm) : sm(sm) {}

ZP_Error SMParamSetup::loadAllParams() {
    ZP_Error result = ZP_ERROR_OK;
    static constexpr ZP_PARAM_ID FLTMODE_PARAMS[SM_FLIGHTMODE_COUNT] = {
        ZP_PARAM_ID::FLTMODE1, ZP_PARAM_ID::FLTMODE2, ZP_PARAM_ID::FLTMODE3,
        ZP_PARAM_ID::FLTMODE4, ZP_PARAM_ID::FLTMODE5, ZP_PARAM_ID::FLTMODE6
    };
    for (uint8_t i = 0; i < SM_FLIGHTMODE_COUNT; i++) {
        float val = 0.0f;
        ZP_Error getResult = ZP_PARAM::get(FLTMODE_PARAMS[i], val);
        result |= getResult;
        if (getResult == ZP_ERROR_OK) {
            sm->flightModes[i] = static_cast<FlightMode_e>(static_cast<uint32_t>(val));
        }
    }

    static constexpr ZP_PARAM_ID RC_REVERSED_PARAMS[SM_RC_REVERSIBLE_COUNT] = {
        ZP_PARAM_ID::RC1_REVERSED, ZP_PARAM_ID::RC2_REVERSED,
        ZP_PARAM_ID::RC3_REVERSED, ZP_PARAM_ID::RC4_REVERSED
    };
    for (uint8_t i = 0; i < SM_RC_REVERSIBLE_COUNT; i++) {
        float val = 0.0f;
        ZP_Error getResult = ZP_PARAM::get(RC_REVERSED_PARAMS[i], val);
        result |= getResult;
        if (getResult == ZP_ERROR_OK) {
            result |= setRCReversed(sm, i, val);
        }
    }

    return result;
}

ZP_Error SMParamSetup::bindAllParamCallbacks() {
    ZP_Error result = ZP_ERROR_OK;
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::FLTMODE1, sm, updateFltMode1);
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::FLTMODE2, sm, updateFltMode2);
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::FLTMODE3, sm, updateFltMode3);
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::FLTMODE4, sm, updateFltMode4);
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::FLTMODE5, sm, updateFltMode5);
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::FLTMODE6, sm, updateFltMode6);

    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::RC1_REVERSED, sm, setRC1Reversed);
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::RC2_REVERSED, sm, setRC2Reversed);
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::RC3_REVERSED, sm, setRC3Reversed);
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::RC4_REVERSED, sm, setRC4Reversed);
    return result;
}

ZP_Error SMParamSetup::setFltMode(SystemManager* ctx, uint8_t idx, float val) {
    if (idx >= SM_FLIGHTMODE_COUNT) return ZP_ERROR_RANGE;

    uint32_t mode = static_cast<uint32_t>(val);

    ZP_Error result = isValidFlightMode(mode);
    if (result != ZP_ERROR_OK) return result;

    ctx->flightModes[idx] = static_cast<FlightMode_e>(mode);
    return ZP_ERROR_OK;
}

ZP_Error SMParamSetup::setRCReversed(SystemManager* ctx, uint8_t idx, float val) {
    if (idx >= SM_RC_REVERSIBLE_COUNT) return ZP_ERROR_RANGE;

    ctx->rcChannelReversed[idx] = (val != 0.0f);
    return ZP_ERROR_OK;
}

// Flightmode param callbacks
ZP_Error SMParamSetup::updateFltMode1(SystemManager* ctx, float val) { return setFltMode(ctx, 0, val); }
ZP_Error SMParamSetup::updateFltMode2(SystemManager* ctx, float val) { return setFltMode(ctx, 1, val); }
ZP_Error SMParamSetup::updateFltMode3(SystemManager* ctx, float val) { return setFltMode(ctx, 2, val); }
ZP_Error SMParamSetup::updateFltMode4(SystemManager* ctx, float val) { return setFltMode(ctx, 3, val); }
ZP_Error SMParamSetup::updateFltMode5(SystemManager* ctx, float val) { return setFltMode(ctx, 4, val); }
ZP_Error SMParamSetup::updateFltMode6(SystemManager* ctx, float val) { return setFltMode(ctx, 5, val); }

// Channel reverse callbacks
ZP_Error SMParamSetup::setRC1Reversed(SystemManager* ctx, float val) { return setRCReversed(ctx, 0, val); }
ZP_Error SMParamSetup::setRC2Reversed(SystemManager* ctx, float val) { return setRCReversed(ctx, 1, val); }
ZP_Error SMParamSetup::setRC3Reversed(SystemManager* ctx, float val) { return setRCReversed(ctx, 2, val); }
ZP_Error SMParamSetup::setRC4Reversed(SystemManager* ctx, float val) { return setRCReversed(ctx, 3, val); }
