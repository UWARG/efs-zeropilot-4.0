#pragma once

// Shared arming/readiness status published by AM and consumed by SM
struct ArmingStatus {
    volatile bool readyToArm = false;
    volatile bool armed = false;
};
