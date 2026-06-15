
#pragma once

#include <cstdint>
#include <cstring>

#include "can_node.hpp"
#include "canard.h"
#include "dronecan_msgs.h"
#include "uavcan.protocol.NodeStatus.h"
#include "stm32l5xx_hal.h"
#include "cmsis_os2.h"
#include "museq.hpp"



class CANController {

private:
	struct DnaAllocationEntry {
		uint8_t unique_id[16];
		uint8_t nodeId;
	};

	enum class DnaStage : int8_t {
		INVALID = 0,
		FIRST_UNIQUE_ID_PART = 1,
		SECOND_UNIQUE_ID_PART = 2,
		FINAL_UNIQUE_ID_PART = 3,
	};

	static constexpr uint8_t NODE_ID = CANARD_MIN_NODE_ID;
	static constexpr uint8_t MAX_ALLOCATION_ENTRIES = 125;
	static constexpr uint8_t UAVCAN_UNIQUE_ID_LENGTH = 16;
	static uint8_t nodeStatusTransferId;
	static uint8_t dnaAllocationTransferId;

	FDCAN_HandleTypeDef *hfdcan;
	CanardInstance canard;

	CanNode canNodes[CANARD_MAX_NODE_ID + 1];
	uint8_t nextAvailableID = CANARD_MIN_NODE_ID + 1;
	DnaAllocationEntry allocationTable[MAX_ALLOCATION_ENTRIES];
	uint8_t allocationCount = 0;

	uavcan_protocol_NodeStatus nodeStatus;
	uint32_t last1HzTick = 0;

	uint8_t dnaCurrentUniqueId[UAVCAN_UNIQUE_ID_LENGTH] = {0};
	uint8_t dnaCurrentUniqueIdLen = 0;
	uint8_t dnaPreferredNodeId = UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ANY_NODE_ID;
	uint32_t dnaLastAcceptedTick = 0;

	void sendNodeStatus();
	void sendCANTx();
	void handleNodeAllocation(CanardRxTransfer* transfer);
	void handleNodeStatus(CanardRxTransfer* transfer);
	static uint8_t dlcToLength(uint32_t dlc);
	int8_t allocateNode();
	int8_t lookupAllocation(const uint8_t unique_id[16]) const;
	bool isNodeIdAllocated(uint8_t nodeId) const;
	void process1HzTasks();
	DnaStage detectDnaRequestStage(const uavcan_protocol_dynamic_node_id_Allocation& msg) const;
	DnaStage getExpectedDnaStage() const;
	void resetDnaInProgress();
	int16_t publishDnaAllocationResponse(uint8_t nodeId, const uint8_t* unique_id, uint8_t unique_id_len);

public:
	CANController(FDCAN_HandleTypeDef *hfdcan);

	bool CanardShouldAcceptTransfer(const CanardInstance* ins,
		uint64_t* out_data_type_signature,
		uint16_t data_type_id,
		CanardTransferType transfer_type,
		uint8_t sourceNodeId);

	void CanardOnTransferReception(CanardInstance* ins,
		CanardRxTransfer* transfer);

	~CANController();

	// Called as much as possible
	bool routineTasks();

	void handleRxFrame(FDCAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data);

	int16_t broadcastObj(
		CanardTxTransfer* transfer
	);

	int16_t broadcast(
		CanardTransferType transfer_type,
		uint64_t data_type_signature,
		uint16_t data_type_id,
		uint8_t* inout_transfer_id,
		uint8_t priority,
		const uint8_t* payload,
		uint16_t payload_len
		#if CANARD_ENABLE_CANFD
			, bool canfd              // True to send as a CAN FD frame
		#endif
		#if CANARD_ENABLE_DEADLINE
			, uint64_t deadline_usec  // Transfer deadline in microseconds
		#endif
		#if CANARD_MULTI_IFACE
			, uint8_t iface_mask      // Bitmask of interfaces to send the transfer on
		#endif
		#if CANARD_ENABLE_TAO_OPTION
			, bool tao                // True to enable tail array optimization
		#endif
	);
};
