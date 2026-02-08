#include <cstdint>
#include "dronecan_msgs.h"

struct canNode {
    uint64_t lastSeenTick;
    uavcan_protocol_NodeStatus status;
};