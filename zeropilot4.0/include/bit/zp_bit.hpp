#pragma once

#include <cstdint>
#include "zp_error.h"
#include "systemutils_iface.hpp"

enum class BitPhase_e : uint8_t {
    POWER_ON,
    CONTINUOUS
};

enum class BitLevel_e : uint8_t {
    WARNING,
    CRITICAL
};

enum class BitState_e : uint8_t {
    UNKNOWN,
    SUCCESS,
    FAILURE
};

enum class ZP_BIT_ID : uint16_t { // NOLINT
    //  POWER_ON
    PARAM_TABLE_INIT,
    IMU_INIT,
    GPS1_INIT,
    GPS2_INIT,
    BARO_INIT,
    RC_INIT,
    PM_INIT,
    TELEM_INIT,
    RANGEFINDER_INIT,
    MOTOR_INIT,
    CAN_INIT,

    // CONTINUOUS
    RC_DATA_VALID,
    IMU_DATA_VALID,
    GPS_DATA_VALID,
    BARO_DATA_VALID,
    PM_DATA_VALID,
    RANGEFINDER_DATA_VALID,
    TELEM_LINK_VALID,
    BATT_LOW,
    BATT_CRITICAL,
    AM_LOOP_TIMING,
    SM_LOOP_TIMING,
    TM_LOOP_TIMING,
    IWDG_REFRESH,
    LOGGER_VALID,

    NUM_BIT_IDS
};

typedef struct {
    const char* name;
    BitPhase_e phase;
    BitLevel_e level;
    uint32_t failMs;
    uint32_t clearMs;
} BitConfig_t;

namespace ZP_BIT {

    ZP_Error init(ISystemUtils* clock);

    /*
    @brief Reports one check. Anything other than ZP_ERROR_OK counts as a failing observation.
    @retval whether the report was recorded, not the health of the BIT
    */
    ZP_Error report(ZP_BIT_ID id, ZP_Error status);

    // Resets every latch to its BIT's current state
    ZP_Error clearLatched();

    /*
    @brief Overrides the table default for how long a failing run must last before the BIT fails
    @param failMs: The time to declare a BIT to be failed
    */
    ZP_Error setFailPersistence(ZP_BIT_ID id, uint32_t failMs);

    /*
    @brief Overrides the table default for how long a passing run must last before the BIT clears
    @param clearMs: The time to declare a BIT to be successful
    */
    ZP_Error setClearPersistence(ZP_BIT_ID id, uint32_t clearMs);

    /*
    @brief Sets both persistence values for a BIT for a state change
    @param failMs: The time to declare a BIT to be failed
    @param clearMs: The time to declare a BIT to be successful
    */
    ZP_Error setPersistence(ZP_BIT_ID id, uint32_t failMs, uint32_t clearMs);

    // Get live state
    ZP_Error getLive(ZP_BIT_ID id, BitState_e& outState);
    
    // Get latched state
    ZP_Error getLatched(ZP_BIT_ID id, BitState_e& outState);

    /*
    @brief Gets the ZP_Error most recently reported for this BIT, ZP_ERROR_OK while it is passing
    @param outError: receives the stored error
    */
    ZP_Error getError(ZP_BIT_ID id, ZP_Error& outError);

    /*
    @brief Gets the name of a BIT
    @param outName: receives the name, left untouched if the id is out of range
    */
    ZP_Error name(ZP_BIT_ID id, const char*& outName);
}

inline constexpr BitConfig_t BIT_CONFIG[static_cast<uint16_t>(ZP_BIT_ID::NUM_BIT_IDS)] = {
    {"PARAM_TABLE_INIT", BitPhase_e::POWER_ON, BitLevel_e::CRITICAL, 0, 0},
    {"IMU_INIT",         BitPhase_e::POWER_ON, BitLevel_e::CRITICAL, 0, 0},
    {"GPS1_INIT",        BitPhase_e::POWER_ON, BitLevel_e::WARNING,  0, 0},
    {"GPS2_INIT",        BitPhase_e::POWER_ON, BitLevel_e::WARNING,  0, 0},
    {"BARO_INIT",        BitPhase_e::POWER_ON, BitLevel_e::WARNING,  0, 0},
    {"RC_INIT",          BitPhase_e::POWER_ON, BitLevel_e::CRITICAL, 0, 0},
    {"PM_INIT",          BitPhase_e::POWER_ON, BitLevel_e::WARNING,  0, 0},
    {"TELEM_INIT",       BitPhase_e::POWER_ON, BitLevel_e::WARNING,  0, 0},
    {"RANGEFINDER_INIT", BitPhase_e::POWER_ON, BitLevel_e::WARNING,  0, 0},
    {"MOTOR_INIT",       BitPhase_e::POWER_ON, BitLevel_e::CRITICAL, 0, 0}, 
    {"CAN_INIT",         BitPhase_e::POWER_ON, BitLevel_e::WARNING,  0, 0},

    {"RC_DATA_VALID",     BitPhase_e::CONTINUOUS, BitLevel_e::CRITICAL, 500,  150},
    {"IMU_DATA_VALID",    BitPhase_e::CONTINUOUS, BitLevel_e::CRITICAL, 50,   50},
    {"GPS_DATA_VALID",    BitPhase_e::CONTINUOUS, BitLevel_e::WARNING,  2000, 500},
    {"BARO_DATA_VALID",   BitPhase_e::CONTINUOUS, BitLevel_e::WARNING,  1000, 500},
    {"PM_DATA_VALID",     BitPhase_e::CONTINUOUS, BitLevel_e::WARNING,  2000, 500},
    {"RNGFND_DATA_VALID", BitPhase_e::CONTINUOUS, BitLevel_e::WARNING,  2000, 500},
    {"TELEM_LINK_VALID",  BitPhase_e::CONTINUOUS, BitLevel_e::WARNING,  3000, 1000},
    {"BATT_LOW",          BitPhase_e::CONTINUOUS, BitLevel_e::WARNING,  0,    0},
    {"BATT_CRITICAL",     BitPhase_e::CONTINUOUS, BitLevel_e::CRITICAL, 0,    0},
    {"AM_LOOP_TIMING",    BitPhase_e::CONTINUOUS, BitLevel_e::WARNING,  2000, 2000},
    {"SM_LOOP_TIMING",    BitPhase_e::CONTINUOUS, BitLevel_e::WARNING,  2000, 2000},
    {"TM_LOOP_TIMING",    BitPhase_e::CONTINUOUS, BitLevel_e::WARNING,  2000, 2000},
    {"IWDG_REFRESH",      BitPhase_e::CONTINUOUS, BitLevel_e::CRITICAL, 0,    0},
    {"LOGGER_VALID",      BitPhase_e::CONTINUOUS, BitLevel_e::WARNING,  1000, 1000},
};
