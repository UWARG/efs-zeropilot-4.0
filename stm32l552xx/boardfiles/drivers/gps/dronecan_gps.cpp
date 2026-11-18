#include "dronecan_gps.hpp"
#include "uavcan.equipment.gnss.Fix2.h"
#include <cmath>
#include <ctime>

DroneCanGPS::DroneCanGPS(CANController *controller) {
    if (!controller) { registration = ZP_Error(ZP_ERROR_NULLPTR); return; }
    mutex = osMutexNew(nullptr);
    if (!mutex) { registration = ZP_Error(ZP_ERROR_RESOURCE_UNAVAILABLE); return; }
    registration = controller->subscribe(this, UAVCAN_EQUIPMENT_GNSS_FIX2_ID,
                                        UAVCAN_EQUIPMENT_GNSS_FIX2_SIGNATURE);
}

ZP_Error DroneCanGPS::onTransfer(CanardRxTransfer *transfer) {
    if (registration != ZP_ERROR_OK) return registration;
    if (!transfer) return ZP_ERROR_NULLPTR;
    if (transfer->data_type_id != UAVCAN_EQUIPMENT_GNSS_FIX2_ID ||
        transfer->transfer_type != CanardTransferTypeBroadcast ||
        transfer->source_node_id == 0) return ZP_ERROR_INVALID_ARG;
    uavcan_equipment_gnss_Fix2 msg{};
    if (uavcan_equipment_gnss_Fix2_decode(transfer, &msg)) return ZP_ERROR_PARSE;
    if (osMutexAcquire(mutex, 0) != osOK) return ZP_ERROR_BUSY;
    // Lock to the first sender; permit a replacement after its samples expire.
    const uint32_t now = HAL_GetTick();
    if (source && source != transfer->source_node_id && now - receivedAt <= STALE_MS) {
        (void)osMutexRelease(mutex);
        return ZP_ERROR_INVALID_DATA;
    }
    source = transfer->source_node_id;
    receivedAt = now;
    valid = false;
    latest.isNew = false;
    if (msg.status < UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_2D_FIX ||
        msg.latitude_deg_1e8 < -9000000000LL || msg.latitude_deg_1e8 > 9000000000LL ||
        msg.longitude_deg_1e8 < -18000000000LL || msg.longitude_deg_1e8 > 18000000000LL ||
        !std::isfinite(msg.ned_velocity[0]) || !std::isfinite(msg.ned_velocity[1]) ||
        !std::isfinite(msg.ned_velocity[2])) {
        (void)osMutexRelease(mutex);
        return ZP_ERROR_INVALID_DATA;
    }
    GpsData_t sample{};
    sample.latitude = static_cast<float>(msg.latitude_deg_1e8 * 1e-8);
    sample.longitude = static_cast<float>(msg.longitude_deg_1e8 * 1e-8);
    sample.altitude = msg.status == UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_3D_FIX
        ? msg.height_msl_mm * 0.001f : INVALID_ALTITUDE;
    sample.numSatellites = msg.sats_used;
    // Match the existing UART NAV-PVT path: north/east/down, metres per second.
    sample.vx = msg.ned_velocity[0];
    sample.vy = msg.ned_velocity[1];
    sample.vz = msg.ned_velocity[2];
    sample.groundSpeed = std::hypot(sample.vx, sample.vy) * 100.0f;
    if (!std::isfinite(sample.groundSpeed)) {
        (void)osMutexRelease(mutex);
        return ZP_ERROR_RANGE;
    }
    sample.trackAngle = INVALID_TRACK_ANGLE;
    if (sample.groundSpeed > 1.0f) {
        sample.trackAngle = std::atan2(sample.vy, sample.vx) * 57.295779513f;
        if (sample.trackAngle < 0) sample.trackAngle += 360.0f;
    }
    // Unknown/non-UTC timestamps remain zero, never pretend they are UTC.
    const uint64_t seconds = msg.gnss_timestamp.usec / 1000000ULL;
    if (msg.gnss_time_standard == UAVCAN_EQUIPMENT_GNSS_FIX2_GNSS_TIME_STANDARD_UTC &&
        seconds > 0 && seconds <= 2147483647ULL) {
        time_t stamp = static_cast<time_t>(seconds);
        struct tm utc{};
        if (gmtime_r(&stamp, &utc)) {
            sample.time = {static_cast<uint16_t>(utc.tm_year + 1900),
                static_cast<uint8_t>(utc.tm_mon + 1), static_cast<uint8_t>(utc.tm_mday),
                static_cast<uint8_t>(utc.tm_hour), static_cast<uint8_t>(utc.tm_min),
                static_cast<uint8_t>(utc.tm_sec)};
        }
    }
    sample.isNew = true;
    latest = sample;
    valid = true;
    (void)osMutexRelease(mutex);
    return ZP_ERROR_OK;
}

ZP_Error DroneCanGPS::readData(GpsData_t &data) {
    data.isNew = false;
    if (registration != ZP_ERROR_OK) return registration;
    if (osMutexAcquire(mutex, 0) != osOK) return ZP_ERROR_BUSY;
    ZP_Error result = ZP_ERROR_NOT_READY;
    if (valid && HAL_GetTick() - receivedAt > STALE_MS) {
        valid = false;
        latest.isNew = false;
        result = ZP_Error(ZP_ERROR_TIMEOUT);
    } else if (valid) {
        data = latest;
        latest.isNew = false;
        result = ZP_Error(ZP_ERROR_OK);
    }
    (void)osMutexRelease(mutex);
    return result;
}
