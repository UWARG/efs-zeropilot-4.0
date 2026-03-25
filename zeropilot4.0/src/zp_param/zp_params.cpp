#include "zp_params.hpp"
#include "mavlink.h"
#include "flightmode.hpp"
#include <cstring>

namespace ZP_PARAM {

// Internal storage hidden from other files using static linkage
static Param_t params[static_cast<uint16_t>(ZP_PARAM_ID::PARAM_COUNT)];

// Internal helper to initialize a single entry
static void initParam(ZP_PARAM_ID id, const char* name, float default_val, uint8_t type) {
    uint16_t index = static_cast<uint16_t>(id);
    if (index >= static_cast<uint16_t>(ZP_PARAM_ID::PARAM_COUNT)) return;

    // Ensure clean string copy and null termination
    std::strncpy(params[index].paramId, name, PARAM_MAX_IDENTIFIER_LEN - 1);
    params[index].paramId[PARAM_MAX_IDENTIFIER_LEN - 1] = '\0';
    
    params[index].paramValue = default_val;
    params[index].paramType = type;
    params[index].context = nullptr;
    params[index].setter = nullptr;
}

void init() {
    std::memset(params, 0, sizeof(params));

    // Define your parameter set
    initParam(ZP_PARAM_ID::RLL2SRV_P, "RLL2SRV_P", 1.120f, MAV_PARAM_TYPE_REAL32);
    initParam(ZP_PARAM_ID::RLL2SRV_I, "RLL2SRV_I", 0.100f, MAV_PARAM_TYPE_REAL32);
    initParam(ZP_PARAM_ID::RLL2SRV_D, "RLL2SRV_D", 0.650f, MAV_PARAM_TYPE_REAL32);
    initParam(ZP_PARAM_ID::RLL2SRV_TAU, "RLL2SRV_TAU", 0.020f, MAV_PARAM_TYPE_REAL32);
    initParam(ZP_PARAM_ID::RLL2SRV_IMAX, "RLL2SRV_IMAX", 50, MAV_PARAM_TYPE_UINT8);

    initParam(ZP_PARAM_ID::PTCH2SRV_P, "PTCH2SRV_P", 2.250f, MAV_PARAM_TYPE_REAL32);
    initParam(ZP_PARAM_ID::PTCH2SRV_I, "PTCH2SRV_I", 0.250f, MAV_PARAM_TYPE_REAL32);
    initParam(ZP_PARAM_ID::PTCH2SRV_D, "PTCH2SRV_D", 1.400f, MAV_PARAM_TYPE_REAL32);
    initParam(ZP_PARAM_ID::PTCH2SRV_TAU, "PTCH2SRV_TAU", 0.020f, MAV_PARAM_TYPE_REAL32);
    initParam(ZP_PARAM_ID::PTCH2SRV_IMAX, "PTCH2SRV_IMAX", 50, MAV_PARAM_TYPE_UINT8);

    initParam(ZP_PARAM_ID::KFF_RDDRMIX, "KFF_RDDRMIX", 0.500f, MAV_PARAM_TYPE_REAL32);

    initParam(ZP_PARAM_ID::ROLL_LIMIT_DEG, "ROLL_LIMIT_DEG", 45.0f, MAV_PARAM_TYPE_REAL32);
    initParam(ZP_PARAM_ID::PTCH_LIM_MAX_DEG, "PTCH_LIM_MAX_DEG", 20.0f, MAV_PARAM_TYPE_REAL32);
    initParam(ZP_PARAM_ID::PTCH_LIM_MIN_DEG, "PTCH_LIM_MIN_DEG", -20.0f, MAV_PARAM_TYPE_REAL32);

    initParam(ZP_PARAM_ID::FLTMODE1, "FLTMODE1", static_cast<float>(PlaneFlightMode_e::MANUAL), MAV_PARAM_TYPE_UINT32);
    initParam(ZP_PARAM_ID::FLTMODE2, "FLTMODE2", static_cast<float>(PlaneFlightMode_e::FBWA),   MAV_PARAM_TYPE_UINT32);
    initParam(ZP_PARAM_ID::FLTMODE3, "FLTMODE3", static_cast<float>(PlaneFlightMode_e::FBWB),   MAV_PARAM_TYPE_UINT32);
    initParam(ZP_PARAM_ID::FLTMODE4, "FLTMODE4", static_cast<float>(PlaneFlightMode_e::MANUAL), MAV_PARAM_TYPE_UINT32);
    initParam(ZP_PARAM_ID::FLTMODE5, "FLTMODE5", static_cast<float>(PlaneFlightMode_e::MANUAL), MAV_PARAM_TYPE_UINT32);
    initParam(ZP_PARAM_ID::FLTMODE6, "FLTMODE6", static_cast<float>(PlaneFlightMode_e::MANUAL), MAV_PARAM_TYPE_UINT32);

    initParam(ZP_PARAM_ID::RC_FS_TIMEOUT, "RC_FS_TIMEOUT", 0.5f, MAV_PARAM_TYPE_REAL32);

    initParam(ZP_PARAM_ID::BATT_LOW_VOLT, "BATT_LOW_VOLT", 10.5f, MAV_PARAM_TYPE_REAL32);
    initParam(ZP_PARAM_ID::BATT_CRT_VOLT, "BATT_CRT_VOLT", 10.2f, MAV_PARAM_TYPE_REAL32);
    initParam(ZP_PARAM_ID::BATT_CAPACITY, "BATT_CAPACITY", 4000.0f, MAV_PARAM_TYPE_REAL32);
    initParam(ZP_PARAM_ID::BATT_LOW_TIMER, "BATT_LOW_TIMER", 5.0f, MAV_PARAM_TYPE_REAL32);

    initParam(ZP_PARAM_ID::AM_FBWB_TOTAL_ENERGY_P_GAIN, "FBWB_TE_P", 1.0f, MAV_PARAM_TYPE_REAL32);
    initParam(ZP_PARAM_ID::AM_FBWB_TOTAL_ENERGY_I_GAIN, "FBWB_TE_I", 0.1f, MAV_PARAM_TYPE_REAL32);
    initParam(ZP_PARAM_ID::AM_FBWB_TOTAL_ENERGY_D_GAIN, "FBWB_TE_D", 0.05f, MAV_PARAM_TYPE_REAL32);
    initParam(ZP_PARAM_ID::AM_FBWB_TOTAL_ENERGY_D_TAU, "FBWB_TE_TAU", 0.1f, MAV_PARAM_TYPE_REAL32);
    initParam(ZP_PARAM_ID::AM_FBWB_ENERGY_BALANCE_P_GAIN, "FBWB_EB_P", 1.0f, MAV_PARAM_TYPE_REAL32);
    initParam(ZP_PARAM_ID::AM_FBWB_ENERGY_BALANCE_I_GAIN, "FBWB_EB_I", 0.1f, MAV_PARAM_TYPE_REAL32);
    initParam(ZP_PARAM_ID::AM_FBWB_ENERGY_BALANCE_D_GAIN, "FBWB_EB_D", 0.05f, MAV_PARAM_TYPE_REAL32);
    initParam(ZP_PARAM_ID::AM_FBWB_ENERGY_BALANCE_D_TAU, "FBWB_EB_TAU", 0.1f, MAV_PARAM_TYPE_REAL32);

}

void bindCallbackInternal(ZP_PARAM_ID id, void* context, ParamSetterCb_t setter) {
    uint16_t index = static_cast<uint16_t>(id);
    if (index < getCount()) {
        params[index].context = context;
        params[index].setter = setter;
    }
}

float get(ZP_PARAM_ID id) {
    uint16_t index = static_cast<uint16_t>(id);
    if (index < static_cast<uint16_t>(ZP_PARAM_ID::PARAM_COUNT)) {
        return params[index].paramValue;
    }
    return 0.0f; // Should never run
}

bool setParamById(const char* paramId, float new_value) {
    for (uint16_t i = 0; i < getCount(); ++i) {
        if (std::strncmp(params[i].paramId, paramId, PARAM_MAX_IDENTIFIER_LEN - 1) == 0) {
            
            // If there's a setter, let it decide if the value is okay first
            if (params[i].setter != nullptr) {
                if (!params[i].setter(params[i].context, new_value)) {
                    return false; // Param change rejected
                }
            }

            // If setter succeeded (or there is no setter), commit to registry
            params[i].paramValue = new_value;
            return true;
        }
    }
    return false;
}

Param_t* getParamByIndex(uint16_t index) {
    if (index < getCount()) {
        return &params[index];
    }
    return nullptr;
}

int16_t getIndexById(const char* paramId) {
    for (uint16_t i = 0; i < getCount(); ++i) {
        if (std::strncmp(params[i].paramId, paramId, PARAM_MAX_IDENTIFIER_LEN - 1) == 0) return i;
    }

    return static_cast<int16_t>(ZP_PARAM_ID::PARAM_COUNT);
}

uint16_t getCount() {
    return static_cast<uint16_t>(ZP_PARAM_ID::PARAM_COUNT);
}

} // namespace ZP_PARAM
