
#include "can_controller.hpp"
#include "drivers.hpp"

static constexpr size_t CANARD_MEMORY_BUFFER_SIZE = 1024;
static constexpr uint32_t CAN_FRAME_EFF_BIT = 31U;

uint8_t CANController::nodeStatusTransferId = 0;
uint8_t CANController::dnaAllocationTransferId = 0;

static void staticOnTransferReception(CanardInstance* ins, CanardRxTransfer* transfer) {
    CANController* self = static_cast<CANController*>(ins->user_reference);
    self->CanardOnTransferReception(ins, transfer);
}

static bool staticShouldAcceptTransfer(const CanardInstance* ins, uint64_t* outSig, uint16_t id, CanardTransferType type, uint8_t src) {
	return static_cast<CANController*>(ins->user_reference)->CanardShouldAcceptTransfer(ins, outSig, id, type, src);
}

CANController::CANController(FDCAN_HandleTypeDef *hfdcan) : hfdcan(hfdcan) {
	static uint8_t canardMemoryPool[CANARD_MEMORY_BUFFER_SIZE];

	canardInit(&canard,
		canardMemoryPool,
		sizeof(canardMemoryPool),
		&staticOnTransferReception,
		&staticShouldAcceptTransfer,
		this
	);

	nodeStatus = {0};

	// All other nodes are default-constructed to OFFLINE.
	canNodes[CANController::NODE_ID].markOnline(systemUtilsHandle->getCurrentTimestampMs());

	canard.node_id = CANController::NODE_ID;

	// Enable bus off interrupt
	HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_BUS_OFF, 0);
}

bool CANController::CanardShouldAcceptTransfer(
    const CanardInstance* ins,
    uint64_t* outDataTypeSignature,
    uint16_t dataTypeId,
    CanardTransferType transferType,
    uint8_t sourceNodeId)
{
    (void)ins;
    (void)sourceNodeId;
    (void)transferType;

    switch (dataTypeId) {
        case UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID: {
            *outDataTypeSignature = UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_SIGNATURE;
            return true;
        }

        case UAVCAN_PROTOCOL_NODESTATUS_ID: {
            *outDataTypeSignature = UAVCAN_PROTOCOL_NODESTATUS_SIGNATURE;
            return true;
        }

        default: {
            return false;
        }
    }
}

void CANController::CanardOnTransferReception(CanardInstance* ins, CanardRxTransfer* transfer) {
    switch (transfer->data_type_id)
    {
        case UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID: {
            if (transfer->transfer_type == CanardTransferTypeBroadcast) {
				handleNodeAllocation(transfer);
            }
            break;
        }

        case UAVCAN_PROTOCOL_NODESTATUS_ID: {
            handleNodeStatus(transfer);
            break;
        }

        default: {
            break;
        }
    }
}

void CANController::handleRxFrame(FDCAN_RxHeaderTypeDef *rxHeader, uint8_t * rxData) {
	const uint64_t timestampUsec = systemUtilsHandle->getCurrentTimestampMs() * 1000ULL;

	CanardCANFrame frame;
	frame.id = rxHeader->Identifier;
	frame.id |= (1UL << CAN_FRAME_EFF_BIT);
	frame.data_len = dlcToLength(rxHeader->DataLength);
	memcpy(frame.data, rxData, frame.data_len);

	canardHandleRxFrame(&canard, &frame, timestampUsec);
}

void CANController::handleNodeStatus(CanardRxTransfer *transfer) {
	uint32_t tick = systemUtilsHandle->getCurrentTimestampMs();

	uavcan_protocol_NodeStatus status {};

	if (uavcan_protocol_NodeStatus_decode(transfer, &status)) return;

	const uint8_t sourceNodeId = transfer->source_node_id;

	// Node ID out of bounds or is anonymous
	if (sourceNodeId > CANARD_MAX_NODE_ID || sourceNodeId == 0) return;

	canNodes[sourceNodeId].update(status, tick);
}

void CANController::handleNodeAllocation(CanardRxTransfer *transfer){

	const uint8_t sourceNodeId = transfer->source_node_id;

	// Only process anonymous requests
	if (sourceNodeId != 0) return;

	uavcan_protocol_dynamic_node_id_Allocation msg;

	if (uavcan_protocol_dynamic_node_id_Allocation_decode(transfer, &msg)) return;

	const uint32_t tick = systemUtilsHandle->getCurrentTimestampMs();

	// If timeout, reset stage
	if (tick > dnaLastAcceptedTick + UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_FOLLOWUP_TIMEOUT_MS) {
		resetDnaInProgress();
	}

	const CANController::DnaStage incoming = detectDnaRequestStage(msg);
	if (incoming == CANController::DnaStage::INVALID || incoming != getExpectedDnaStage()) {
		return;
	}
	
	// Append the new chunk
	memcpy(dnaCurrentUniqueId + dnaCurrentUniqueIdLen, msg.unique_id.data, msg.unique_id.len);
	dnaCurrentUniqueIdLen += msg.unique_id.len;

	if (incoming == CANController::DnaStage::FIRST_UNIQUE_ID_PART) {
		dnaPreferredNodeId = msg.node_id;
	}

	if (dnaCurrentUniqueIdLen == UAVCAN_UNIQUE_ID_LENGTH) {
		const int8_t newNodeId = allocateNode();
		if (newNodeId >= 0) {
			(void)publishDnaAllocationResponse(newNodeId, dnaCurrentUniqueId, dnaCurrentUniqueIdLen);
		}
		resetDnaInProgress();
	} else {
		if (publishDnaAllocationResponse(0, dnaCurrentUniqueId, dnaCurrentUniqueIdLen) < 0) {
			resetDnaInProgress();
			return;
		}
		dnaLastAcceptedTick = tick;
	}
}

int8_t CANController::lookupAllocation(const uint8_t uniqueId[16]) const {
	for (uint8_t i = 0; i < allocationCount; i++) {
		if (memcmp(allocationTable[i].uniqueId, uniqueId, 16) == 0) {
			return allocationTable[i].nodeId;
		}
	}
	return -1;
}

bool CANController::isNodeIdAllocated(uint8_t nodeId) const {
	for (uint8_t i = 0; i < allocationCount; i++) {
		if (allocationTable[i].nodeId == nodeId) {
			return true;
		}
	}
	return false;
}

int8_t CANController::allocateNode() {
	// DroneCAN reserves 126 and 127
	static constexpr uint8_t MAX_DYNAMIC_NODE_ID = 125;

	// Check if previously assigned
	int8_t existingId = lookupAllocation(dnaCurrentUniqueId);
	if (existingId > 0) {
		return existingId;
	}

	// Try preferred ID
	int8_t assignedId = 0;
	if (dnaPreferredNodeId != UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ANY_NODE_ID &&
		dnaPreferredNodeId >= CANARD_MIN_NODE_ID &&
		dnaPreferredNodeId <= MAX_DYNAMIC_NODE_ID &&
		canNodes[dnaPreferredNodeId].isOffline() &&
		!isNodeIdAllocated(dnaPreferredNodeId)) {
		assignedId = dnaPreferredNodeId;
	}

	// Scan for free ID
	if (assignedId == 0) {
		for (int currId = nextAvailableID; currId <= MAX_DYNAMIC_NODE_ID && assignedId == 0; currId++) {
			if (canNodes[currId].isOffline()
				&& !isNodeIdAllocated(currId)) {
				nextAvailableID = currId + 1;
				assignedId = currId;
			}
		}

		if (assignedId == 0) {
			return -1;
		}
	}

	// Push to allocation table
	if (allocationCount < MAX_ALLOCATION_ENTRIES) {
		memcpy(allocationTable[allocationCount].uniqueId, dnaCurrentUniqueId, 16);
		allocationTable[allocationCount].nodeId = assignedId;
		allocationCount++;
	}

	return assignedId;
}

CANController::DnaStage CANController::detectDnaRequestStage(const uavcan_protocol_dynamic_node_id_Allocation& msg) const {

	constexpr uint8_t MAX_LEN = UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_LENGTH_OF_UNIQUE_ID_IN_REQUEST;
	constexpr uint8_t STAGE3_LEN = UAVCAN_UNIQUE_ID_LENGTH - MAX_LEN * 2U;

	const uint8_t len = msg.unique_id.len;

	// Length should be 6 or 4
	if (len != MAX_LEN && len != STAGE3_LEN) {
		return CANController::DnaStage::INVALID;
	}

	if (msg.first_part_of_unique_id) {
		return CANController::DnaStage::FIRST_UNIQUE_ID_PART;
	}

	if (len == MAX_LEN) {
		return CANController::DnaStage::SECOND_UNIQUE_ID_PART;
	}

	if (len == STAGE3_LEN) {
		return CANController::DnaStage::FINAL_UNIQUE_ID_PART;
	}

	return CANController::DnaStage::INVALID;
}

CANController::DnaStage CANController::getExpectedDnaStage() const {
	constexpr uint8_t MAX_LEN = UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_LENGTH_OF_UNIQUE_ID_IN_REQUEST;

	switch (dnaCurrentUniqueIdLen) {
		case 0: 
			return CANController::DnaStage::FIRST_UNIQUE_ID_PART;
		case MAX_LEN:
			return CANController::DnaStage::SECOND_UNIQUE_ID_PART;
		case MAX_LEN * 2:
			return CANController::DnaStage::FINAL_UNIQUE_ID_PART;
		default:
			return CANController::DnaStage::INVALID;
	}
}


void CANController::resetDnaInProgress() {
	dnaCurrentUniqueIdLen = 0;
	dnaPreferredNodeId = UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ANY_NODE_ID;
	dnaLastAcceptedTick = 0;
}

int16_t CANController::publishDnaAllocationResponse(uint8_t nodeId, const uint8_t* unique_id, uint8_t unique_id_len) {
	uavcan_protocol_dynamic_node_id_Allocation msg {};
	msg.node_id = nodeId;
	msg.first_part_of_unique_id = false;
	msg.unique_id.len = unique_id_len;
	memcpy(msg.unique_id.data, unique_id, unique_id_len);

	uint8_t buffer[UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_SIZE];
	uint32_t len = uavcan_protocol_dynamic_node_id_Allocation_encode(&msg, buffer);

	return broadcast(
		CanardTransferTypeBroadcast,
		UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_SIGNATURE,
		UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID,
		&dnaAllocationTransferId,
		CANARD_TRANSFER_PRIORITY_LOW,
		buffer,
		len
	);
}

void CANController::sendCANTx() {
	CanardCANFrame* frame = canardPeekTxQueue(&canard);
	if (frame == nullptr) return;

	if (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) > 0) {
		FDCAN_TxHeaderTypeDef txHeader;
		txHeader.Identifier = frame->id & CANARD_CAN_EXT_ID_MASK;
		txHeader.IdType = FDCAN_EXTENDED_ID;
		txHeader.TxFrameType = FDCAN_DATA_FRAME;
		txHeader.DataLength = lengthToDlc(frame->data_len);
		txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
		txHeader.BitRateSwitch = FDCAN_BRS_OFF;
		txHeader.FDFormat = FDCAN_CLASSIC_CAN;
		txHeader.TxEventFifoControl = FDCAN_STORE_TX_EVENTS;
		txHeader.MessageMarker = 3;

		uint8_t txData[8];
		memcpy(txData, frame->data, frame->data_len);

		if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &txHeader, txData) == HAL_OK) {
			canardPopTxQueue(&canard);
		}
	}
}

bool CANController::routineTasks() {
	sendCANTx();

	uint32_t tick = systemUtilsHandle->getCurrentTimestampMs();

	if (tick > last1HzTick + UAVCAN_PROTOCOL_NODESTATUS_MAX_BROADCASTING_PERIOD_MS / 2) {
		last1HzTick = tick;
		process1HzTasks();
	}

	return true;
}

void CANController::sendNodeStatus() {
	uint8_t buffer[UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_MAX_SIZE];

    nodeStatus.uptime_sec = systemUtilsHandle->getCurrentTimestampMs() / 1000LL;
    nodeStatus.health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
    nodeStatus.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;
    nodeStatus.sub_mode = 0;
    nodeStatus.vendor_specific_status_code = 1234;

    uint32_t len = uavcan_protocol_NodeStatus_encode(&nodeStatus, buffer);

    broadcast(CanardTransferTypeBroadcast,
			UAVCAN_PROTOCOL_NODESTATUS_SIGNATURE,
			UAVCAN_PROTOCOL_NODESTATUS_ID,
			&nodeStatusTransferId,
			CANARD_TRANSFER_PRIORITY_LOW,
			buffer,
			len
	);

}

void CANController::process1HzTasks() {

	uint32_t timestampMsec = systemUtilsHandle->getCurrentTimestampMs();

	// Mark remote nodes offline if they have not been seen recently
	for (int i = CANARD_MIN_NODE_ID; i <= CANARD_MAX_NODE_ID; i++) {
		if (i != CANController::NODE_ID) {
			canNodes[i].updateLiveness(timestampMsec, UAVCAN_PROTOCOL_NODESTATUS_OFFLINE_TIMEOUT_MS);
		}
	}

	// Transmit NodeStatus
	sendNodeStatus();
}


int16_t CANController::broadcastObj(CanardTxTransfer* transfer) {
	return canardBroadcastObj(&canard, transfer);
}

int16_t CANController::broadcast(
	CanardTransferType transferType,
	uint64_t dataTypeSignature,
	uint16_t dataTypeId,
	uint8_t* inoutTransferId,
	uint8_t priority,
	const uint8_t* payload,
	uint16_t payloadLen
	#if CANARD_ENABLE_CANFD
		, bool canfd
	#endif
	#if CANARD_ENABLE_DEADLINE
		, uint64_t deadlineUsec
	#endif
	#if CANARD_MULTI_IFACE
		, uint8_t ifaceMask
	#endif
	#if CANARD_ENABLE_TAO_OPTION
		, bool tao
	#endif
)
{
	CanardTxTransfer transfer_object;
	transfer_object.transfer_type = transferType;
	transfer_object.data_type_signature = dataTypeSignature;
	transfer_object.data_type_id = dataTypeId;
	transfer_object.inout_transfer_id = inoutTransferId;
	transfer_object.priority = priority;
	transfer_object.payload = payload;
	transfer_object.payload_len = payloadLen;
	
	#if CANARD_ENABLE_CANFD
		transfer_object.canfd = canfd;
	#endif
	#if CANARD_ENABLE_DEADLINE
		transfer_object.deadline_usec = deadlineUsec;
	#endif
	#if CANARD_MULTI_IFACE
		transfer_object.iface_mask = ifaceMask;
	#endif
	#if CANARD_ENABLE_TAO_OPTION
		transfer_object.tao = tao;
	#endif

	return broadcastObj(&transfer_object);
}

uint8_t CANController::dlcToLength(uint32_t dlc) {
	switch (dlc) {
		case FDCAN_DLC_BYTES_0: return 0;
		case FDCAN_DLC_BYTES_1: return 1;
		case FDCAN_DLC_BYTES_2: return 2;
		case FDCAN_DLC_BYTES_3: return 3;
		case FDCAN_DLC_BYTES_4: return 4;
		case FDCAN_DLC_BYTES_5: return 5;
		case FDCAN_DLC_BYTES_6: return 6;
		case FDCAN_DLC_BYTES_7: return 7;
		case FDCAN_DLC_BYTES_8: return 8;
		default: return 0;
	}
}

uint32_t CANController::lengthToDlc(uint8_t len) {
    switch (len) {
        case 0: return FDCAN_DLC_BYTES_0;
        case 1: return FDCAN_DLC_BYTES_1;
        case 2: return FDCAN_DLC_BYTES_2;
        case 3: return FDCAN_DLC_BYTES_3;
        case 4: return FDCAN_DLC_BYTES_4;
        case 5: return FDCAN_DLC_BYTES_5;
        case 6: return FDCAN_DLC_BYTES_6;
        case 7: return FDCAN_DLC_BYTES_7;
        case 8: return FDCAN_DLC_BYTES_8;
        default: return FDCAN_DLC_BYTES_0;
    }
}
