#include "sm_param_setup.hpp"
#include "system_manager.hpp"
#include "zp_params.hpp"
#include "flightmode.hpp"

SMParamSetup::SMParamSetup(SystemManager* sm) : sm(sm) {}

void SMParamSetup::loadAllParams() {
    static constexpr ZP_PARAM_ID FLTMODE_PARAMS[SM_FLIGHTMODE_COUNT] = {
        ZP_PARAM_ID::FLTMODE1, ZP_PARAM_ID::FLTMODE2, ZP_PARAM_ID::FLTMODE3,
        ZP_PARAM_ID::FLTMODE4, ZP_PARAM_ID::FLTMODE5, ZP_PARAM_ID::FLTMODE6
    };
    for (uint8_t i = 0; i < SM_FLIGHTMODE_COUNT; i++) {
        sm->flightModes[i] = static_cast<PlaneFlightMode_e>(
            static_cast<uint32_t>(ZP_PARAM::get(FLTMODE_PARAMS[i])));
    }
    setRC1Reversed(sm, ZP_PARAM::get(ZP_PARAM_ID::RC1_REVERSED));
    setRC2Reversed(sm, ZP_PARAM::get(ZP_PARAM_ID::RC2_REVERSED));
    setRC3Reversed(sm, ZP_PARAM::get(ZP_PARAM_ID::RC3_REVERSED));
    setRC4Reversed(sm, ZP_PARAM::get(ZP_PARAM_ID::RC4_REVERSED));
}

void SMParamSetup::bindAllParamCallbacks() {
    ZP_PARAM::bindCallback(ZP_PARAM_ID::FLTMODE1, sm, updateFltMode1);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::FLTMODE2, sm, updateFltMode2);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::FLTMODE3, sm, updateFltMode3);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::FLTMODE4, sm, updateFltMode4);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::FLTMODE5, sm, updateFltMode5);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::FLTMODE6, sm, updateFltMode6);

    ZP_PARAM::bindCallback(ZP_PARAM_ID::RC1_REVERSED, sm, setRC1Reversed);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::RC2_REVERSED, sm, setRC2Reversed);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::RC3_REVERSED, sm, setRC3Reversed);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::RC4_REVERSED, sm, setRC4Reversed);
}

bool SMParamSetup::setFltMode(SystemManager* ctx, uint8_t idx, float val) {
    uint32_t mode = static_cast<uint32_t>(val);
    if (!isValidPlaneFlightMode(mode)) return false;
    ctx->flightModes[idx] = static_cast<PlaneFlightMode_e>(mode);
    return true;
}

bool SMParamSetup::updateFltMode1(SystemManager* ctx, float val) { return setFltMode(ctx, 0, val); }
bool SMParamSetup::updateFltMode2(SystemManager* ctx, float val) { return setFltMode(ctx, 1, val); }
bool SMParamSetup::updateFltMode3(SystemManager* ctx, float val) { return setFltMode(ctx, 2, val); }
bool SMParamSetup::updateFltMode4(SystemManager* ctx, float val) { return setFltMode(ctx, 3, val); }
bool SMParamSetup::updateFltMode5(SystemManager* ctx, float val) { return setFltMode(ctx, 4, val); }
bool SMParamSetup::updateFltMode6(SystemManager* ctx, float val) { return setFltMode(ctx, 5, val); }
bool SMParamSetup::setRC1Reversed(SystemManager* ctx, float val) { ctx->rcChannelReversed[0] = (val != 0.0f); return true; }
bool SMParamSetup::setRC2Reversed(SystemManager* ctx, float val) { ctx->rcChannelReversed[1] = (val != 0.0f); return true; }
bool SMParamSetup::setRC3Reversed(SystemManager* ctx, float val) { ctx->rcChannelReversed[2] = (val != 0.0f); return true; }
bool SMParamSetup::setRC4Reversed(SystemManager* ctx, float val) { ctx->rcChannelReversed[3] =( val != 0.0f); return true; }
