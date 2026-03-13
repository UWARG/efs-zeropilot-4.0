#include "zp_params.hpp"
#include "mavlink.h"
#include <cstring>

namespace ZP_PARAM {

// Internal storage hidden from other files using static linkage
static Param_t params[static_cast<uint16_t>(ZP_PARAM_ID::PARAM_COUNT)];

// Internal helper to initialize a single entry
static void initSingleParam(ZP_PARAM_ID id, const char* name, float default_val, uint8_t type) {
    uint16_t index = static_cast<uint16_t>(id);
    if (index >= static_cast<uint16_t>(ZP_PARAM_ID::PARAM_COUNT)) return;

    // Ensure clean string copy and null termination
    std::strncpy(params[index].param_id, name, PARAM_MAX_IDENTIFIER_LEN - 1);
    params[index].param_id[PARAM_MAX_IDENTIFIER_LEN - 1] = '\0';
    
    params[index].param_value = default_val;
    params[index].param_type = type;
    params[index].context = nullptr;
    params[index].setter = nullptr;
}

void init() {
    std::memset(params, 0, sizeof(params));

    // Define your parameter set
    initSingleParam(ZP_PARAM_ID::PID_ROLL_KP,   "PID_ROLL_KP",   1.120f, MAV_PARAM_TYPE_REAL32);
    initSingleParam(ZP_PARAM_ID::PID_ROLL_KI,   "PID_ROLL_KI",   0.100f, MAV_PARAM_TYPE_REAL32);
    initSingleParam(ZP_PARAM_ID::PID_ROLL_KD,   "PID_ROLL_KD",   0.650f, MAV_PARAM_TYPE_REAL32);
    initSingleParam(ZP_PARAM_ID::PID_ROLL_TAU,  "PID_ROLL_TAU",  0.020f, MAV_PARAM_TYPE_REAL32);
    initSingleParam(ZP_PARAM_ID::PID_PITCH_KP,  "PID_PITCH_KP",  2.250f, MAV_PARAM_TYPE_REAL32);
    initSingleParam(ZP_PARAM_ID::PID_PITCH_KI,  "PID_PITCH_KI",  0.250f, MAV_PARAM_TYPE_REAL32);
    initSingleParam(ZP_PARAM_ID::PID_PITCH_KD,  "PID_PITCH_KD",  1.400f, MAV_PARAM_TYPE_REAL32);
    initSingleParam(ZP_PARAM_ID::PID_PITCH_TAU, "PID_PITCH_TAU", 0.020f, MAV_PARAM_TYPE_REAL32);
    initSingleParam(ZP_PARAM_ID::KFF_RDDRMIX,   "KFF_RDDRMIX",   0.500f, MAV_PARAM_TYPE_REAL32);
}

void bindCallback(ZP_PARAM_ID id, void* context, ParamSetterCb setter) {
    uint16_t index = static_cast<uint16_t>(id);
    if (index < getCount()) {
        params[index].context = context;
        params[index].setter = setter;
    }
}

float get(ZP_PARAM_ID id) {
    uint16_t index = static_cast<uint16_t>(id);
    if (index < static_cast<uint16_t>(ZP_PARAM_ID::PARAM_COUNT)) {
        return params[index].param_value;
    }
    return 0.0f; // Should never run
}

bool setParamById(const char* param_id, float new_value) {
    for (uint16_t i = 0; i < getCount(); ++i) {
        if (std::strncmp(params[i].param_id, param_id, PARAM_MAX_IDENTIFIER_LEN) == 0) {
            params[i].param_value = new_value;

            // Notify any registered class instance of the update (e.g., PID controller)
            if (params[i].setter != nullptr) {
                params[i].setter(params[i].context, new_value);
            }
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

int16_t getIndexById(const char* param_id) {
    for (uint16_t i = 0; i < getCount(); ++i) {
        if (strcmp(params[i].param_id, param_id) == 0) return i;
    }
    return static_cast<int16_t>(ZP_PARAM_ID::PARAM_COUNT);
}

uint16_t getCount() {
    return static_cast<uint16_t>(ZP_PARAM_ID::PARAM_COUNT);
}

} // namespace ZP_PARAM
