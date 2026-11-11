#include "can_opflow.hpp"

#include "drivers.hpp"
#include "uavcan.equipment.range_sensor.Measurement.h"
#include "FreeRTOS.h"
#include "task.h"

CanOpflow::CanOpflow(ISystemUtils *systemUtils) : systemUtils(systemUtils) {
    // Needs to be constructed after canControllerHandle in initDrivers()
    if (canControllerHandle == nullptr) {
        subscribeStatus |= ZP_ERROR_NULLPTR;
        return;
    }

    // The H-Flow broadcasts two messages, subscribe to both
    subscribeStatus |= canControllerHandle->subscribe(this, UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_ID, UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_SIGNATURE);
    subscribeStatus |= canControllerHandle->subscribe(this, FLOW_MSG_ID, FLOW_MSG_SIGNATURE);
}

ZP_Error CanOpflow::init() {
    // Nothing to configure on the H-Flow itself, CANController assigns its node ID.
    // Report whether both subscriptions went through.
    return subscribeStatus;
}

ZP_Error CanOpflow::onTransfer(CanardRxTransfer *transfer) {
    if (transfer == nullptr) {
        return ZP_ERROR_NULLPTR;
    }

    // The H-Flow only broadcasts, ignore requests/responses
    if (transfer->transfer_type != CanardTransferTypeBroadcast) {
        return ZP_ERROR_UNSUPPORTED;
    }

    switch (transfer->data_type_id) {
        case UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_ID:
            return decodeRange(transfer);
        case FLOW_MSG_ID:
            return decodeFlow(transfer);
        default:
            return ZP_ERROR_UNSUPPORTED;
    }
}

ZP_Error CanOpflow::decodeRange(CanardRxTransfer *transfer) {
    uavcan_equipment_range_sensor_Measurement msg = {};

    // Generated decode functions return true when the payload is invalid
    if (uavcan_equipment_range_sensor_Measurement_decode(transfer, &msg)) {
        return ZP_ERROR_PARSE;
    }

    RangefinderData_t newData = {};
    newData.distance = msg.range;
    newData.signalStrength = 0; // Not sent by the H-Flow
    newData.temp = 0; // Not sent by the H-Flow
    // Too close / too far readings still come through, but the distance isn't usable
    newData.isValid = (msg.reading_type == UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_READING_TYPE_VALID_RANGE);
    newData.isNew = true;

    uint32_t now = systemUtils->getCurrentTimestampMs();

    // Written from the bus thread and read from another thread, so don't let a half written sample be read
    taskENTER_CRITICAL();
    rangeData = newData;
    lastRangeMs = now;
    taskEXIT_CRITICAL();

    return ZP_ERROR_OK;
}

ZP_Error CanOpflow::decodeFlow(CanardRxTransfer *transfer) {
    if (transfer->payload_len < FLOW_MSG_SIZE) {
        return ZP_ERROR_PARSE;
    }

    // Layout: float32 integration_interval, float32[2] rate_gyro_integral,
    //         float32[2] flow_integral, uint8 quality
    OpticalFlowData_t newData = {};
    canardDecodeScalar(transfer, 0, 32, true, &newData.integrationInterval);
    canardDecodeScalar(transfer, 32, 32, true, &newData.gyroX);
    canardDecodeScalar(transfer, 64, 32, true, &newData.gyroY);
    canardDecodeScalar(transfer, 96, 32, true, &newData.flowX);
    canardDecodeScalar(transfer, 128, 32, true, &newData.flowY);
    canardDecodeScalar(transfer, 160, 8, false, &newData.quality);

    // TODO: ask if we want a minimum quality here as well
    newData.isValid = (newData.integrationInterval > 0.0f);
    newData.isNew = true;

    uint32_t now = systemUtils->getCurrentTimestampMs();

    taskENTER_CRITICAL();
    flowData = newData;
    lastFlowMs = now;
    taskEXIT_CRITICAL();

    return ZP_ERROR_OK;
}

ZP_Error CanOpflow::readData(RangefinderData_t &data) {
    taskENTER_CRITICAL();
    data = rangeData;
    uint32_t lastMs = lastRangeMs;
    rangeData.isNew = false;
    taskEXIT_CRITICAL();

    // Nothing received yet
    if (lastMs == 0) {
        data.isValid = false;
        return ZP_ERROR_NOT_READY;
    }

    // Sensor stopped sending
    if (systemUtils->getCurrentTimestampMs() - lastMs > OPFLOW_TIMEOUT_MS) {
        data.isValid = false;
        return ZP_ERROR_TIMEOUT;
    }

    return data.isValid ? ZP_ERROR_OK : ZP_ERROR_INVALID_DATA;
}

ZP_Error CanOpflow::readFlowData(OpticalFlowData_t &data) {
    taskENTER_CRITICAL();
    data = flowData;
    uint32_t lastMs = lastFlowMs;
    flowData.isNew = false;
    taskEXIT_CRITICAL();

    if (lastMs == 0) {
        data.isValid = false;
        return ZP_ERROR_NOT_READY;
    }

    if (systemUtils->getCurrentTimestampMs() - lastMs > OPFLOW_TIMEOUT_MS) {
        data.isValid = false;
        return ZP_ERROR_TIMEOUT;
    }

    return data.isValid ? ZP_ERROR_OK : ZP_ERROR_INVALID_DATA;
}
