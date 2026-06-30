#include "direct_mapping.hpp"
#include "drivers.hpp"
#include "managers.hpp"

// Pre-allocated static storage (global, not stack)
alignas(AttitudeManager) static uint8_t amHandleStorage[sizeof(AttitudeManager)];
alignas(SystemManager) static uint8_t smHandleStorage[sizeof(SystemManager)];
alignas(TelemetryManager) static uint8_t tmHandleStorage[sizeof(TelemetryManager)];
alignas(ExMemManager) static uint8_t emHandleStorage[sizeof(ExMemManager)];

// Manager handles
AttitudeManager *amHandle = nullptr;
SystemManager *smHandle = nullptr;
TelemetryManager *tmHandle = nullptr;
ExMemManager *emHandle = nullptr;

void initManagers()
{
    // AM initialization
    amHandle = new (&amHandleStorage) AttitudeManager(
        systemUtilsHandle, 
        gpsHandle,
        imuHandle,
        amRCQueueHandle,
        tmQueueHandle,
        &mainMotorGroup
    );

    // SM initialization
    smHandle = new (&smHandleStorage) SystemManager(
        systemUtilsHandle, 
        iwdgHandle,
        sdFileSystemHandle,
        rcHandle,
		pmHandle,
        amRCQueueHandle,
        tmQueueHandle
    );

    // TM initialization
    tmHandle = new (&tmHandleStorage) TelemetryManager(
        systemUtilsHandle,
        telemLinkHandle,
        tmQueueHandle,
        amRCQueueHandle,
        messageBufferHandle
    );

    // EMM initialization
    emHandle = new (&emHandleStorage) ExMemManager(
        systemUtilsHandle,
        fatFsBackendHandle,
        sdRequestQueueHandle,
        sdBufferQueueHandle,
        sdResponseQueuesHandle
    );
}
