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
    PID_ROLL_KP = 0,
    PID_ROLL_KI,
    PID_ROLL_KD,
    PID_ROLL_TAU,
    PID_PITCH_KP,
    PID_PITCH_KI,
    PID_PITCH_KD,
    PID_PITCH_TAU,
    KFF_RDDRMIX,
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
