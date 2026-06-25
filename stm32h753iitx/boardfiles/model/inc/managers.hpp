#pragma once

#include "attitude_manager.hpp"
#include "system_manager.hpp"
#include "telemetry_manager.hpp"
#include "fatfs_manager.hpp"

extern AttitudeManager *amHandle;
extern SystemManager *smHandle;
extern TelemetryManager *tmHandle;
extern FatFSManager *ffmHandle;

void initManagers();
