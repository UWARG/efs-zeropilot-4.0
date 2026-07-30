
#pragma once

#include <cstdint>
#include <cstring>

#include "can_node.hpp"
#include "canard.h"
#include "dronecan_msgs.h"
#include "uavcan.protocol.NodeStatus.h"
#include "stm32h7xx_hal.h"
#include "cmsis_os2.h"
#include "museq.hpp"
#include "systemutils.hpp"

class CANController {
private:
	SystemUtils *systemutilsDriver;
	
    uint8_t profilerId;
	

	struct DnaAllocationEntry {
		uint8_t uniqueId[16];
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

	struct RawCanFrame {
		uint32_t id;
		uint8_t dlc; 
		uint8_t data[8];
	};

	static constexpr uint32_t CAN_RX_QUEUE_CAPACITY = 32;
	static constexpr uint32_t CAN_RX_RING_SLOTS = CAN_RX_QUEUE_CAPACITY + 1;
	RawCanFrame canRxRing[CAN_RX_RING_SLOTS] {};
	volatile uint32_t canRxHead = 0; // write idx
	volatile uint32_t canRxTail = 0; // read idx

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
	int8_t allocateNode();
	int8_t lookupAllocation(const uint8_t unique_id[16]) const;
	bool isNodeIdAllocated(uint8_t nodeId) const;
	void process1HzTasks();
	DnaStage detectDnaRequestStage(const uavcan_protocol_dynamic_node_id_Allocation& msg) const;
	DnaStage getExpectedDnaStage() const;
	void resetDnaInProgress();
	int16_t publishDnaAllocationResponse(uint8_t nodeId, const uint8_t* unique_id, uint8_t unique_id_len);
	
	static uint8_t dlcToLength(uint32_t dlc);
	static uint32_t lengthToDlc(uint8_t length);
	bool popRxFrame(RawCanFrame *frame);
	void handleRxFrame(const RawCanFrame &frame);

public:
	CANController(FDCAN_HandleTypeDef *hfdcan, SystemUtils *systemutilsDriver);
	
	~CANController() = delete;

	bool CanardShouldAcceptTransfer(const CanardInstance* ins,
		uint64_t* outDataTypeSignature,
		uint16_t dataTypeId,
		CanardTransferType transferType,
		uint8_t sourceNodeId);

	void CanardOnTransferReception(CanardInstance* ins,
		CanardRxTransfer* transfer);

	// Called as much as possible
	bool routineTasks();

	bool enqueueRxFrame(uint32_t id, uint32_t dlc, const uint8_t *data);

	int16_t broadcastObj(
		CanardTxTransfer* transfer
	);

	int16_t broadcast(
		CanardTransferType transferType,
		uint64_t dataTypeSignature,
		uint16_t dataTypeId,
		uint8_t* inoutTransferId,
		uint8_t priority,
		const uint8_t* payload,
		uint16_t payloadLen
		#if CANARD_ENABLE_CANFD
			, bool canfd              // True to send as a CAN FD frame
		#endif
		#if CANARD_ENABLE_DEADLINE
			, uint64_t deadlineUsec  // Transfer deadline in microseconds
		#endif
		#if CANARD_MULTI_IFACE
			, uint8_t ifaceMask      // Bitmask of interfaces to send the transfer on
		#endif
		#if CANARD_ENABLE_TAO_OPTION
			, bool tao                // True to enable tail array optimization
		#endif
	);
};
