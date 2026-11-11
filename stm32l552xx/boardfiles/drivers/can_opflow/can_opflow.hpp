#pragma once

#include <cstdint>

#include "canard.h"
#include "can_listener_iface.hpp"
#include "rangefinder_iface.hpp"
#include "systemutils_iface.hpp"
#include "zp_error.h"

// com.hex.equipment.flow.Measurement isn't in our generated DroneCAN files, so its
// ID, signature and size come from its DSDL definition and it is decoded by hand
static constexpr uint16_t FLOW_MSG_ID = 20200;
static constexpr uint64_t FLOW_MSG_SIGNATURE = 0x6A908866BCB49C18ULL;
static constexpr uint16_t FLOW_MSG_SIZE = 21; // 5 x float32 + 1 x uint8

// H-Flow sends much faster than this, so no data for this long means it dropped out
static constexpr uint32_t OPFLOW_TIMEOUT_MS = 500;

// Optical flow sample from the H-Flow
typedef struct {
    float integrationInterval; // s, time the values below were integrated over
    float flowX; // rad, integrated optical flow about the sensor X axis
    float flowY; // rad, integrated optical flow about the sensor Y axis
    float gyroX; // rad, integrated gyro about the sensor X axis
    float gyroY; // rad, integrated gyro about the sensor Y axis
    uint8_t quality; // 0 to 255, higher is better
    bool isValid; // True if data is valid, false otherwise
    bool isNew; // True if the data is new since last readFlowData()
} OpticalFlowData_t;

// Holybro H-Flow (optical flow + ToF rangefinder) over DroneCAN.
// Subscribes to the H-Flow's range and flow messages on CANController, which calls
// onTransfer() from the bus thread. The latest samples are kept for readData() / readFlowData().
class CanOpflow : public ICanListener, public IRangefinder {
    public:
        CanOpflow(ISystemUtils *systemUtils);

        ZP_Error init() override;
        ZP_Error readData(RangefinderData_t &data) override;
        ZP_Error readFlowData(OpticalFlowData_t &data);

        ZP_Error onTransfer(CanardRxTransfer *transfer) override;

    private:
        ISystemUtils *systemUtils;
        ZP_Error subscribeStatus = ZP_ERROR_OK; // Result of subscribing in the constructor, reported by init()

        RangefinderData_t rangeData = {};
        OpticalFlowData_t flowData = {};
        uint32_t lastRangeMs = 0;
        uint32_t lastFlowMs = 0;

        ZP_Error decodeRange(CanardRxTransfer *transfer);
        ZP_Error decodeFlow(CanardRxTransfer *transfer);
};
