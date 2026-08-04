
#include "can_node.hpp"

CanNode::CanNode() {
	status.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_OFFLINE;
}

void CanNode::update(const uavcan_protocol_NodeStatus& newStatus, uint64_t tick) {
	status = newStatus;
	lastSeenTick = tick;
}

void CanNode::markOnline(uint64_t tick) {
	status.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;
	lastSeenTick = tick;
}

void CanNode::updateLiveness(uint64_t now, uint64_t timeoutMs) {
	if (now - lastSeenTick > timeoutMs) {
		status.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_OFFLINE;
	}
}

bool CanNode::isOffline() const {
	return status.mode == UAVCAN_PROTOCOL_NODESTATUS_MODE_OFFLINE;
}

const uavcan_protocol_NodeStatus& CanNode::getStatus() const {
	return status;
}
