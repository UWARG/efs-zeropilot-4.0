#pragma once

#include "error.h"
#include "attitude_manager.hpp"
#include "system_manager.hpp"
#include "telemetry_manager.hpp"

extern AttitudeManager *amHandle;
extern SystemManager *smHandle;
extern TelemetryManager *tmHandle;

ZP_ERROR_e initManagers();
