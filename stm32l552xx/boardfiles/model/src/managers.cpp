#include "direct_mapping.hpp"
#include "drivers.hpp"
#include "managers.hpp"

// Pre-allocated static storage (global, not stack)
alignas(AttitudeManager) static uint8_t amHandleStorage[sizeof(AttitudeManager)];
alignas(SystemManager) static uint8_t smHandleStorage[sizeof(SystemManager)];
alignas(TelemetryManager) static uint8_t tmHandleStorage[sizeof(TelemetryManager)];

// Manager handles
AttitudeManager *amHandle = nullptr;
SystemManager *smHandle = nullptr;
TelemetryManager *tmHandle = nullptr;

void initManagers()
{
    // AM initialization
    amHandle = new (&amHandleStorage) AttitudeManager(
        systemUtilsHandle, 
        gpsHandle,
        imuHandle,
        amRCQueueHandle, 
        tmQueueHandle, 
        smLoggerQueueHandle, 
        &rollMotors, 
        &pitchMotors, 
        &yawMotors, 
        &throttleMotors, 
        &flapMotors, 
        &steeringMotors
    );

    // SM initialization
    smHandle = new (&smHandleStorage) SystemManager(
        systemUtilsHandle, 
        iwdgHandle,
        loggerHandle,
        rcHandle,
		pmHandle,
        amRCQueueHandle,
        tmQueueHandle,
        smLoggerQueueHandle
    );

    // TM initialization
    tmHandle = new (&tmHandleStorage) TelemetryManager(
        systemUtilsHandle,
        rfdHandle,
        tmQueueHandle,
        amRCQueueHandle,
        messageBufferHandle
    );
}
