#include "direct_mapping.hpp"
#include "drivers.hpp"
#include "managers.hpp"
#include "error.h"

AttitudeManager *amHandle = nullptr;
SystemManager *smHandle = nullptr;
TelemetryManager *tmHandle = nullptr;
DirectMapping *flightMode = nullptr;

ZP_ERROR_e initManagers()
{
    // AM initialization
    flightMode = new DirectMapping();
    if (flightMode == nullptr) {
      return ZP_ERROR_OUT_OF_MEMORY;
    }

    amHandle = new AttitudeManager(systemUtilsHandle, gpsHandle, amRCQueueHandle, tmQueueHandle, smLoggerQueueHandle, flightMode, &rollMotors, &pitchMotors, &yawMotors, &throttleMotors, &flapMotors, &steeringMotors);
    if (amHandle == nullptr) {
      return ZP_ERROR_OUT_OF_MEMORY;
    }

    // SM initialization
    smHandle = new SystemManager(systemUtilsHandle, iwdgHandle, loggerHandle, rcHandle, amRCQueueHandle, tmQueueHandle, smLoggerQueueHandle);
    if (smHandle == nullptr) {
      return ZP_ERROR_OUT_OF_MEMORY;
    }

    // TM initialization
    tmHandle = new TelemetryManager(systemUtilsHandle, rfdHandle, tmQueueHandle, amRCQueueHandle, messageBufferHandle);
    if (tmHandle == nullptr) {
      return ZP_ERROR_OUT_OF_MEMORY;
    }

    return ZP_ERROR_OK;
}
