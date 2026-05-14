#include <cstdint>
#include "dronecan_msgs.h"

struct canNode {
    uint64_t lastSeenTick;
    uavcan_protocol_NodeStatus status;
};

struct DnaAllocationEntry {
    uint8_t unique_id[16];
    uint8_t node_id;
};

enum class DnaStage : int8_t {
    INVALID = 0,
    STAGE_1 = 1,
    STAGE_2 = 2,
    STAGE_3 = 3,
};