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

// Shared arming/readiness state: written by AM, read by SM
static ArmingStatus armingStatus;

void initManagers()
{
    // AM initialization
    amHandle = new (&amHandleStorage) AttitudeManager(
        systemUtilsHandle,
        mathUtilsHandle,
        gpsHandle,
        imuHandle,
        fftHandle,
        rangefinderHandle,
        barometerHandle,
        amRCQueueHandle, 
        tmQueueHandle, 
        smLoggerQueueHandle, 
        &mainMotorGroup,
        &armingStatus
    );

    // SM initialization
    smHandle = new (&smHandleStorage) SystemManager(
        systemUtilsHandle, 
        iwdgHandle,
        loggerHandle,
        nullptr, // safetySwitchHandle: No safety switch on L5, pass nullptr
        rcHandle,
        pmHandle,
        amRCQueueHandle,
        tmQueueHandle,
        smLoggerQueueHandle,
        &armingStatus
    );

    // TM initialization
    tmHandle = new (&tmHandleStorage) TelemetryManager(
        systemUtilsHandle,
        telemLinkHandle,
        tmQueueHandle,
        amRCQueueHandle,
        messageBufferHandle
    );
}
