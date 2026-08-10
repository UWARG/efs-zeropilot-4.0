#pragma once

#include <cstdint>

enum class PrearmReason : uint8_t {
    NONE = 0,          // Ready to arm
    BARO_NOT_SETTLED,  // Barometer home altitude not yet settled
    GPS_NOT_SETTLED,   // GPS home not converged (only gates modes that require GPS)
};

// Shared arming/readiness status published by AM and consumed by SM
struct ArmingStatus {
    volatile bool readyToArm = false;
    volatile bool armed = false;
    volatile PrearmReason prearmReason = PrearmReason::NONE;
};
