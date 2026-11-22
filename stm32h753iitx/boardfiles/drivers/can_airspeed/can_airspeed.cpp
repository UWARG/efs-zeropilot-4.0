#include "can_airspeed.hpp"
#include "drivers.hpp"
#include "uavcan.equipment.air_data.IndicatedAirspeed.h"
#include "FreeRTOS.h"
#include "task.h"

CanAirspeed::CanAirspeed(ISystemUtils *systemUtils) : systemUtils(systemUtils) {
    if (canControllerHandle == nullptr) { 
        subscribeStatus |= ZP_ERROR_NULLPTR;
        return; 
    }

    // Subscribe to the controller
    subscribeStatus |= canControllerHandle->subscribe(this, UAVCAN_EQUIPMENT_AIR_DATA_INDICATEDAIRSPEED_ID, UAVCAN_EQUIPMENT_AIR_DATA_INDICATEDAIRSPEED_SIGNATURE);
}

ZP_Error CanAirspeed::init() {
    return subscribeStatus; 
}

ZP_Error CanAirspeed::onTransfer(CanardRxTransfer *transfer) {
    if(transfer == nullptr) {
        return ZP_ERROR_NULLPTR; // Will be used by subscribe status later
    }

    if (transfer->transfer_type != CanardTransferTypeBroadcast) {
        return ZP_ERROR_UNSUPPORTED; // Airspeed ALSO only broadcasts
    }

    if(transfer->data_type_id == UAVCAN_EQUIPMENT_AIR_DATA_INDICATEDAIRSPEED_ID){
        return decodeAirspeed(transfer);
    } else {
        return ZP_ERROR_UNSUPPORTED;
    }
}

ZP_Error CanAirspeed::decodeAirspeed(CanardRxTransfer *transfer) {
    uavcan_equipment_air_data_IndicatedAirspeed msg = {};

    // 1. Extract airspeed data from transfer
    if(uavcan_equipment_air_data_IndicatedAirspeed_decode(transfer, &msg)){
        return ZP_ERROR_PARSE; 
    }

    // 2. Fetch time to determine if data is new
    uint32_t time = systemUtils->getCurrentTimestampMs();

    AirspeedData_t newData = {};

    newData.speed = msg.indicated_airspeed;
    newData.variance = msg.indicated_airspeed_variance;
    newData.isNew = true;
    newData.isValid = true;
    // Maybe there will be some other criteria for is valid idk

    taskENTER_CRITICAL();
    airspeedData = newData;
    lastAirSpeedMs = time;
    taskEXIT_CRITICAL();

    return ZP_ERROR_OK; 
}

ZP_Error CanAirspeed::readAirspeedData(AirspeedData_t &data) {
    taskENTER_CRITICAL();
    data = airspeedData;
    uint32_t lastTime = lastAirSpeedMs;
    airspeedData.isNew = false;
    taskEXIT_CRITICAL();

    if (lastTime == 0){
        data.isValid = false; 
        return ZP_ERROR_NOT_READY;
    }

    // Assuming that the sensor has stopped sending data
    if (systemUtils->getCurrentTimestampMs() - lastTime > AIRSPEED_TIMEOUT) {
        data.isValid = false; 
        return ZP_ERROR_TIMEOUT;
    }

    return data.isValid ? ZP_ERROR_OK : ZP_ERROR_INVALID_DATA;
}