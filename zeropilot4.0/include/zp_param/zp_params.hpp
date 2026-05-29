#pragma once

#include <cstdint>

#define PARAM_MAX_IDENTIFIER_LEN 17

// Function pointer signature for parameter callbacks
typedef bool (*ParamSetterCb_t)(void* context, float newValue);

typedef struct {
    char paramId[PARAM_MAX_IDENTIFIER_LEN];
    float paramValue;
    uint8_t paramType;
    
    // Callback routing
    void* context;        // Pointer to the object instance
    ParamSetterCb_t setter; // Function pointer to trigger when updated
} Param_t;

// Enumeration for indexing into the global parameter array
enum class ZP_PARAM_ID : uint16_t { // NOLINT
    SERVO1_TRIM = 0,
    SERVO1_MIN,
    SERVO1_MAX,
    SERVO1_REVERSED,
    SERVO1_FUNCTION,
    SERVO2_TRIM,
    SERVO2_MIN,
    SERVO2_MAX,
    SERVO2_REVERSED,
    SERVO2_FUNCTION,
    SERVO3_TRIM,
    SERVO3_MIN,
    SERVO3_MAX,
    SERVO3_REVERSED,
    SERVO3_FUNCTION,
    SERVO4_TRIM,
    SERVO4_MIN,
    SERVO4_MAX,
    SERVO4_REVERSED,
    SERVO4_FUNCTION,
    SERVO5_TRIM,
    SERVO5_MIN,
    SERVO5_MAX,
    SERVO5_REVERSED,
    SERVO5_FUNCTION,
    SERVO6_TRIM,
    SERVO6_MIN,
    SERVO6_MAX,
    SERVO6_REVERSED,
    SERVO6_FUNCTION,
    SERVO7_TRIM,
    SERVO7_MIN,
    SERVO7_MAX,
    SERVO7_REVERSED,
    SERVO7_FUNCTION,
    SERVO8_TRIM,
    SERVO8_MIN,
    SERVO8_MAX,
    SERVO8_REVERSED,
    SERVO8_FUNCTION,
    SERVO9_TRIM,
    SERVO9_MIN,
    SERVO9_MAX,
    SERVO9_REVERSED,
    SERVO9_FUNCTION,
    SERVO10_TRIM,
    SERVO10_MIN,
    SERVO10_MAX,
    SERVO10_REVERSED,
    SERVO10_FUNCTION,
    SERVO11_TRIM,
    SERVO11_MIN,
    SERVO11_MAX,
    SERVO11_REVERSED,
    SERVO11_FUNCTION,
    SERVO12_TRIM,
    SERVO12_MIN,
    SERVO12_MAX,
    SERVO12_REVERSED,
    SERVO12_FUNCTION,
    RLL2SRV_P,
    RLL2SRV_I,
    RLL2SRV_D,
    RLL2SRV_TAU,
    RLL2SRV_IMAX,
    PTCH2SRV_P,
    PTCH2SRV_I,
    PTCH2SRV_D,
    PTCH2SRV_TAU,
    PTCH2SRV_IMAX,
    KFF_RDDRMIX,
    ROLL_LIMIT_DEG,
    PTCH_LIM_MAX_DEG,
    PTCH_LIM_MIN_DEG,
    FLTMODE1,
    FLTMODE2,
    FLTMODE3,
    FLTMODE4,
    FLTMODE5,
    FLTMODE6,
    RC_FS_TIMEOUT,
    BATT_LOW_VOLT,
    BATT_CRT_VOLT,
    BATT_CAPACITY,
    BATT_LOW_TIMER,
    PARAM_COUNT
};

namespace ZP_PARAM {
    // Initialize the registry (call once during system boot)
    void init();

    // Bind a callback to a specific parameter
    void bindCallbackInternal(ZP_PARAM_ID id, void* context, ParamSetterCb_t setter);

    // Templated wrapper for bindCallbackInternal
    template <typename T>
    void bindCallback(ZP_PARAM_ID id, T* context, bool (*setter)(T*, float)) {
        bindCallbackInternal(id, static_cast<void*>(context), reinterpret_cast<ParamSetterCb_t>(setter));
    }

    // Get current config value
    float get(ZP_PARAM_ID id);

    // MAVLink/Telemetry interaction
    bool setParamById(const char* paramId, float new_value);
    
    // Accessors
    Param_t* getParamByIndex(uint16_t index);
    int16_t getIndexById(const char* paramId);
    uint16_t getCount();
}
