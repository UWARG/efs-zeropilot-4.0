#pragma once

#include "gps_iface.hpp"
#include "can_listener_iface.hpp"
#include "can_controller.hpp"
#include "cmsis_os2.h"

// Registered listeners must live for the lifetime of CANController.
class DroneCanGPS : public IGPS, public ICanListener {
public:
    explicit DroneCanGPS(CANController *controller);
    ZP_Error init() const { return registration; }
    ZP_Error onTransfer(CanardRxTransfer *transfer) override;
    ZP_Error readData(GpsData_t &data) override;
    DroneCanGPS(const DroneCanGPS&) = delete;
    DroneCanGPS& operator=(const DroneCanGPS&) = delete;
private:
    osMutexId_t mutex = nullptr;
    ZP_Error registration = ZP_ERROR_NOT_READY;
    GpsData_t latest{};
    uint32_t receivedAt = 0;
    uint8_t source = 0;
    bool valid = false;
    static constexpr uint32_t STALE_MS = 2000;
};
