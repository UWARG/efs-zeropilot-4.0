#pragma once

#include <cstdint>

#include "canard.h"
#include "can_listener_iface.hpp"
#include "rangefinder_iface.hpp"
#include "systemutils_iface.hpp"   // getCurrentTimestampMs()
#include "zp_error.h"   

// not in our generated dronecan files, decoded by hand
static constexpr uint16_t FLOW_MSG_ID = 20200;                          // h-flow data type ID
static constexpr uint64_t FLOW_MSG_SIGNATURE = 0x6A908866BCB49C18ULL;
static constexpr uint16_t FLOW_MSG_SIZE = 21; // 5 x float32 + 1 x uint8

static constexpr uint32_t OPFLOW_TIMEOUT_MS = 500; // no data for this long = sensor dropped out

typedef struct {
    float integrationInterval; // s

    float flowX; // rad
    float flowY; // rad
    float gyroX; // rad
    float gyroY; // rad

    uint8_t quality; // need to find min acceptable value

    bool isValid;
    bool isNew; // since last readFlowData()
} OpticalFlowData_t;

class CanOpflow : public ICanListener, public IRangefinder {
    public:
        CanOpflow(ISystemUtils *systemUtils);

        // IRangeFinder
        ZP_Error init() override;
        ZP_Error readData(RangefinderData_t &data) override;
        ZP_Error readFlowData(OpticalFlowData_t &data);

        // ICanListener
        ZP_Error onTransfer(CanardRxTransfer *transfer) override;

    private:
        ISystemUtils *systemUtils;
        ZP_Error subscribeStatus = ZP_ERROR_OK; // set in constructor, returned by init()

        RangefinderData_t rangeData = {};
        OpticalFlowData_t flowData = {};
        uint32_t lastRangeMs = 0;
        uint32_t lastFlowMs = 0;

        ZP_Error decodeRange(CanardRxTransfer *transfer);
        ZP_Error decodeFlow(CanardRxTransfer *transfer);
};
