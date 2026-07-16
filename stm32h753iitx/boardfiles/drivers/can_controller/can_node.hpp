
#pragma once

#include <cstdint>

#include "uavcan.protocol.NodeStatus.h"

class CanNode {
private:
	uint64_t lastSeenTick = 0;
	uavcan_protocol_NodeStatus status{};
public:
	CanNode();

	void update(const uavcan_protocol_NodeStatus& newStatus, uint64_t tick);
	void markOnline(uint64_t tick);
	void updateLiveness(uint64_t now, uint64_t timeoutMs);
	bool isOffline() const;
	const uavcan_protocol_NodeStatus& getStatus() const;
};
