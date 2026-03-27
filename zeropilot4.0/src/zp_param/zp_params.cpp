#include "zp_params.hpp"
#include "mavlink.h"
#include "flightmode.hpp"
#include "motor_functions.hpp"
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
    initParam(ZP_PARAM_ID::SERVO1_TRIM, "SERVO1_TRIM", 1500, MAV_PARAM_TYPE_UINT16);
    initParam(ZP_PARAM_ID::SERVO1_MIN, "SERVO1_MIN", 1000, MAV_PARAM_TYPE_UINT16);
    initParam(ZP_PARAM_ID::SERVO1_MAX, "SERVO1_MAX", 2000, MAV_PARAM_TYPE_UINT16);
    initParam(ZP_PARAM_ID::SERVO1_REVERSED, "SERVO1_REVERSED", 0, MAV_PARAM_TYPE_UINT8);
    initParam(ZP_PARAM_ID::SERVO1_FUNCTION, "SERVO1_FUNCTION", static_cast<float>(MotorFunction_e::Aileron), MAV_PARAM_TYPE_INT16);

    initParam(ZP_PARAM_ID::SERVO2_TRIM, "SERVO2_TRIM", 1500, MAV_PARAM_TYPE_UINT16);
    initParam(ZP_PARAM_ID::SERVO2_MIN, "SERVO2_MIN", 1000, MAV_PARAM_TYPE_UINT16);
    initParam(ZP_PARAM_ID::SERVO2_MAX, "SERVO2_MAX", 2000, MAV_PARAM_TYPE_UINT16);
    initParam(ZP_PARAM_ID::SERVO2_REVERSED, "SERVO2_REVERSED", 0, MAV_PARAM_TYPE_UINT8);
    initParam(ZP_PARAM_ID::SERVO2_FUNCTION, "SERVO2_FUNCTION", static_cast<float>(MotorFunction_e::Elevator), MAV_PARAM_TYPE_INT16);

    initParam(ZP_PARAM_ID::SERVO3_TRIM, "SERVO3_TRIM", 1500, MAV_PARAM_TYPE_UINT16);
    initParam(ZP_PARAM_ID::SERVO3_MIN, "SERVO3_MIN", 1000, MAV_PARAM_TYPE_UINT16);
    initParam(ZP_PARAM_ID::SERVO3_MAX, "SERVO3_MAX", 2000, MAV_PARAM_TYPE_UINT16);
    initParam(ZP_PARAM_ID::SERVO3_REVERSED, "SERVO3_REVERSED", 0, MAV_PARAM_TYPE_UINT8);
    initParam(ZP_PARAM_ID::SERVO3_FUNCTION, "SERVO3_FUNCTION", static_cast<float>(MotorFunction_e::Throttle), MAV_PARAM_TYPE_INT16);

    initParam(ZP_PARAM_ID::SERVO4_TRIM, "SERVO4_TRIM", 1500, MAV_PARAM_TYPE_UINT16);
    initParam(ZP_PARAM_ID::SERVO4_MIN, "SERVO4_MIN", 1000, MAV_PARAM_TYPE_UINT16);
    initParam(ZP_PARAM_ID::SERVO4_MAX, "SERVO4_MAX", 2000, MAV_PARAM_TYPE_UINT16);
    initParam(ZP_PARAM_ID::SERVO4_REVERSED, "SERVO4_REVERSED", 0, MAV_PARAM_TYPE_UINT8);
    initParam(ZP_PARAM_ID::SERVO4_FUNCTION, "SERVO4_FUNCTION", static_cast<float>(MotorFunction_e::Rudder), MAV_PARAM_TYPE_INT16);

    initParam(ZP_PARAM_ID::SERVO5_TRIM, "SERVO5_TRIM", 1500, MAV_PARAM_TYPE_UINT16);
    initParam(ZP_PARAM_ID::SERVO5_MIN, "SERVO5_MIN", 1000, MAV_PARAM_TYPE_UINT16);
    initParam(ZP_PARAM_ID::SERVO5_MAX, "SERVO5_MAX", 2000, MAV_PARAM_TYPE_UINT16);
    initParam(ZP_PARAM_ID::SERVO5_REVERSED, "SERVO5_REVERSED", 0, MAV_PARAM_TYPE_UINT8);
    initParam(ZP_PARAM_ID::SERVO5_FUNCTION, "SERVO5_FUNCTION", static_cast<float>(MotorFunction_e::Aileron), MAV_PARAM_TYPE_INT16);

    initParam(ZP_PARAM_ID::SERVO6_TRIM, "SERVO6_TRIM", 1500, MAV_PARAM_TYPE_UINT16);
    initParam(ZP_PARAM_ID::SERVO6_MIN, "SERVO6_MIN", 1000, MAV_PARAM_TYPE_UINT16);
    initParam(ZP_PARAM_ID::SERVO6_MAX, "SERVO6_MAX", 2000, MAV_PARAM_TYPE_UINT16);
    initParam(ZP_PARAM_ID::SERVO6_REVERSED, "SERVO6_REVERSED", 0, MAV_PARAM_TYPE_UINT8);
    initParam(ZP_PARAM_ID::SERVO6_FUNCTION, "SERVO6_FUNCTION", static_cast<float>(MotorFunction_e::Flap), MAV_PARAM_TYPE_INT16);

    initParam(ZP_PARAM_ID::SERVO7_TRIM, "SERVO7_TRIM", 1500, MAV_PARAM_TYPE_UINT16);
    initParam(ZP_PARAM_ID::SERVO7_MIN, "SERVO7_MIN", 1000, MAV_PARAM_TYPE_UINT16);
    initParam(ZP_PARAM_ID::SERVO7_MAX, "SERVO7_MAX", 2000, MAV_PARAM_TYPE_UINT16);
    initParam(ZP_PARAM_ID::SERVO7_REVERSED, "SERVO7_REVERSED", 0, MAV_PARAM_TYPE_UINT8);
    initParam(ZP_PARAM_ID::SERVO7_FUNCTION, "SERVO7_FUNCTION", static_cast<float>(MotorFunction_e::Flap), MAV_PARAM_TYPE_INT16);

    initParam(ZP_PARAM_ID::SERVO8_TRIM, "SERVO8_TRIM", 1500, MAV_PARAM_TYPE_UINT16);
    initParam(ZP_PARAM_ID::SERVO8_MIN, "SERVO8_MIN", 1000, MAV_PARAM_TYPE_UINT16);
    initParam(ZP_PARAM_ID::SERVO8_MAX, "SERVO8_MAX", 2000, MAV_PARAM_TYPE_UINT16);
    initParam(ZP_PARAM_ID::SERVO8_REVERSED, "SERVO8_REVERSED", 0, MAV_PARAM_TYPE_UINT8);
    initParam(ZP_PARAM_ID::SERVO8_FUNCTION, "SERVO8_FUNCTION", static_cast<float>(MotorFunction_e::GroundSteering), MAV_PARAM_TYPE_INT16);

    initParam(ZP_PARAM_ID::SERVO9_TRIM, "SERVO9_TRIM", 1500, MAV_PARAM_TYPE_UINT16);
    initParam(ZP_PARAM_ID::SERVO9_MIN, "SERVO9_MIN", 1000, MAV_PARAM_TYPE_UINT16);
    initParam(ZP_PARAM_ID::SERVO9_MAX, "SERVO9_MAX", 2000, MAV_PARAM_TYPE_UINT16);
    initParam(ZP_PARAM_ID::SERVO9_REVERSED, "SERVO9_REVERSED", 0, MAV_PARAM_TYPE_UINT8);
    initParam(ZP_PARAM_ID::SERVO9_FUNCTION, "SERVO9_FUNCTION", static_cast<float>(MotorFunction_e::Disabled), MAV_PARAM_TYPE_INT16);

    initParam(ZP_PARAM_ID::SERVO10_TRIM, "SERVO10_TRIM", 1500, MAV_PARAM_TYPE_UINT16);
    initParam(ZP_PARAM_ID::SERVO10_MIN, "SERVO10_MIN", 1000, MAV_PARAM_TYPE_UINT16);
    initParam(ZP_PARAM_ID::SERVO10_MAX, "SERVO10_MAX", 2000, MAV_PARAM_TYPE_UINT16);
    initParam(ZP_PARAM_ID::SERVO10_REVERSED, "SERVO10_REVERSED", 0, MAV_PARAM_TYPE_UINT8);
    initParam(ZP_PARAM_ID::SERVO10_FUNCTION, "SERVO10_FUNCTION", static_cast<float>(MotorFunction_e::Disabled), MAV_PARAM_TYPE_INT16);

    initParam(ZP_PARAM_ID::SERVO11_TRIM, "SERVO11_TRIM", 1500, MAV_PARAM_TYPE_UINT16);
    initParam(ZP_PARAM_ID::SERVO11_MIN, "SERVO11_MIN", 1000, MAV_PARAM_TYPE_UINT16);
    initParam(ZP_PARAM_ID::SERVO11_MAX, "SERVO11_MAX", 2000, MAV_PARAM_TYPE_UINT16);
    initParam(ZP_PARAM_ID::SERVO11_REVERSED, "SERVO11_REVERSED", 0, MAV_PARAM_TYPE_UINT8);
    initParam(ZP_PARAM_ID::SERVO11_FUNCTION, "SERVO11_FUNCTION", static_cast<float>(MotorFunction_e::Disabled), MAV_PARAM_TYPE_INT16);

    initParam(ZP_PARAM_ID::SERVO12_TRIM, "SERVO12_TRIM", 1500, MAV_PARAM_TYPE_UINT16);
    initParam(ZP_PARAM_ID::SERVO12_MIN, "SERVO12_MIN", 1000, MAV_PARAM_TYPE_UINT16);
    initParam(ZP_PARAM_ID::SERVO12_MAX, "SERVO12_MAX", 2000, MAV_PARAM_TYPE_UINT16);
    initParam(ZP_PARAM_ID::SERVO12_REVERSED, "SERVO12_REVERSED", 0, MAV_PARAM_TYPE_UINT8);
    initParam(ZP_PARAM_ID::SERVO12_FUNCTION, "SERVO12_FUNCTION", static_cast<float>(MotorFunction_e::Disabled), MAV_PARAM_TYPE_INT16);

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
    initParam(ZP_PARAM_ID::FLTMODE3, "FLTMODE3", static_cast<float>(PlaneFlightMode_e::MANUAL), MAV_PARAM_TYPE_UINT32);
    initParam(ZP_PARAM_ID::FLTMODE4, "FLTMODE4", static_cast<float>(PlaneFlightMode_e::MANUAL), MAV_PARAM_TYPE_UINT32);
    initParam(ZP_PARAM_ID::FLTMODE5, "FLTMODE5", static_cast<float>(PlaneFlightMode_e::MANUAL), MAV_PARAM_TYPE_UINT32);
    initParam(ZP_PARAM_ID::FLTMODE6, "FLTMODE6", static_cast<float>(PlaneFlightMode_e::MANUAL), MAV_PARAM_TYPE_UINT32);

    initParam(ZP_PARAM_ID::RC_FS_TIMEOUT, "RC_FS_TIMEOUT", 0.5f, MAV_PARAM_TYPE_REAL32);

    initParam(ZP_PARAM_ID::BATT_LOW_VOLT, "BATT_LOW_VOLT", 10.5f, MAV_PARAM_TYPE_REAL32);
    initParam(ZP_PARAM_ID::BATT_CRT_VOLT, "BATT_CRT_VOLT", 10.2f, MAV_PARAM_TYPE_REAL32);
    initParam(ZP_PARAM_ID::BATT_CAPACITY, "BATT_CAPACITY", 4000.0f, MAV_PARAM_TYPE_REAL32);
    initParam(ZP_PARAM_ID::BATT_LOW_TIMER, "BATT_LOW_TIMER", 5.0f, MAV_PARAM_TYPE_REAL32);
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
