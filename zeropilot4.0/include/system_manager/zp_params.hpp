#pragma once

#include <cstdint>

#define PARAM_MAX_IDENTIFIER_LEN 17

// Function pointer signature for parameter callbacks
typedef void (*ParamSetterCb)(void* context, float newValue);

typedef struct {
    char param_id[PARAM_MAX_IDENTIFIER_LEN];
    float param_value;
    uint8_t param_type;
    
    // Callback routing
    void* context;        // Pointer to the object instance
    ParamSetterCb setter; // Function pointer to trigger when updated
} Param_t;

// Enumeration for indexing into the global parameter array
enum class ZP_PARAM_ID : uint16_t {
    PID_ROLL_KP = 0,
    PID_ROLL_KI,
    PID_ROLL_KD,
    PID_ROLL_TAU,
    PID_PITCH_KP,
    PID_PITCH_KI,
    PID_PITCH_KD,
    PID_PITCH_TAU,
    KFF_RDDRMIX,
    PARAM_COUNT
};

namespace ZP_PARAM {
    // Initialize the registry (call once during system boot)
    void init();

    // Bind a callback to a specific parameter
    void bindCallback(ZP_PARAM_ID id, void* context, ParamSetterCb setter);

    // Get current config value
    float get(ZP_PARAM_ID id);

    // MAVLink/Telemetry interaction
    bool setParamById(const char* param_id, float new_value);
    
    // Accessors
    Param_t* getParamByIndex(uint16_t index);
    int16_t getIndexById(const char* param_id);
    uint16_t getCount();
}
