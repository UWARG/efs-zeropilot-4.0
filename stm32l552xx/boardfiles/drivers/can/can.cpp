
#include "can.hpp"

uint8_t CAN::transfer_id = 0;

static void StaticOnTransferReception(CanardInstance* ins, CanardRxTransfer* transfer) {
    CAN* self = static_cast<CAN*>(ins->user_reference);
    self->CanardOnTransferReception(ins, transfer);
}

static bool StaticShouldAcceptTransfer(const CanardInstance* ins, uint64_t* out_sig, uint16_t id, CanardTransferType type, uint8_t src) {

	return static_cast<CAN*>(ins->user_reference)->CanardShouldAcceptTransfer(ins, out_sig, id, type, src);
}

CAN::CAN(FDCAN_HandleTypeDef *hfdcan) : hfdcan(hfdcan) {
	static uint8_t canardMemoryPool[1024];

	canardInit(&canard,
			canardMemoryPool,
			sizeof(canardMemoryPool),
			&StaticOnTransferReception,
			&StaticShouldAcceptTransfer,
			this
	);

	nodeStatus = {0};

	for (int i = 0; i <= CANARD_MAX_NODE_ID; i++) {
		canNodes[i].status.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_OFFLINE;
	}
	canNodes[NODE_ID].status.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;

	canard.node_id = NODE_ID;
}

CAN::~CAN() {}



bool CAN::CanardShouldAcceptTransfer(
    const CanardInstance* ins,
    uint64_t* out_data_type_signature,
    uint16_t data_type_id,
    CanardTransferType transfer_type,
    uint8_t source_node_id)
{
    (void)ins;
    (void)source_node_id;

    switch (data_type_id)
    {
        case UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID:
            *out_data_type_signature = UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_SIGNATURE;
            return true;

        case UAVCAN_PROTOCOL_NODESTATUS_ID:
            *out_data_type_signature = UAVCAN_PROTOCOL_NODESTATUS_SIGNATURE;
            return true;

        default:
            return false;
    }
}

void CAN::CanardOnTransferReception(CanardInstance* ins, CanardRxTransfer* transfer)
{

    switch (transfer->data_type_id)
    {
        case UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID:
        {
            if (transfer->transfer_type == CanardTransferTypeBroadcast)
            {
				handleNodeAllocation(transfer);
            }

            break;
        }

        case UAVCAN_PROTOCOL_NODESTATUS_ID:
        {
            handleNodeStatus(transfer);
            break;
        }

        default:
            break;
    }
}

uint8_t CAN::dlcToLength(uint32_t dlc) {
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
		default: return 8; // fallback
	}
}


void CAN::handleRxFrame(FDCAN_RxHeaderTypeDef *rx_header, uint8_t * rx_data) {
	const uint64_t timestamp_usec = HAL_GetTick() * 1000ULL;

	CanardCANFrame frame;
	frame.id = rx_header->Identifier;
	frame.id |= (1UL << 31U); // Add EFF bit
	frame.data_len = dlcToLength(rx_header->DataLength);
	memcpy(frame.data, rx_data, frame.data_len);

	canardHandleRxFrame(&canard, &frame, timestamp_usec);
}

void CAN::handleNodeStatus(CanardRxTransfer *transfer) {
	uint32_t tick = HAL_GetTick();

	canNode node {0};

	node.lastSeenTick = tick;

	bool invalid = uavcan_protocol_NodeStatus_decode(transfer, &node.status);

	if (invalid) return;

	// Node ID out of bounds or is anonymous
	if (transfer->source_node_id > CANARD_MAX_NODE_ID) return;
	if (transfer->source_node_id == 0) return;

	canNodes[transfer->source_node_id] = node;
}

void CAN::handleNodeAllocation(CanardRxTransfer *transfer){

	//only process anonymous requests
	if (transfer->source_node_id != 0) return;

	uavcan_protocol_dynamic_node_id_Allocation msg;

	if (uavcan_protocol_dynamic_node_id_Allocation_decode(transfer, &msg)) return;

	const uint32_t tick = HAL_GetTick();

	// if timeout reset stage
	if (tick > dnaLastAcceptedTick + UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_FOLLOWUP_TIMEOUT_MS) {
		resetDnaInProgress();
	}

	const DnaStage incoming = detectDnaRequestStage(msg);
	if (incoming == DnaStage::INVALID)
		return;
	if (incoming != getExpectedDnaStage())
		return;
	// append the new chunk
	memcpy(dnaCurrentUniqueId + dnaCurrentUniqueIdLen, msg.unique_id.data, msg.unique_id.len);
	dnaCurrentUniqueIdLen += msg.unique_id.len;


	if (incoming == DnaStage::STAGE_1) {
		dnaPreferredNodeId = msg.node_id;
	}

	if (dnaCurrentUniqueIdLen == UAVCAN_UNIQUE_ID_LENGTH) {
		const int8_t new_node_id = allocateNode();
		if (new_node_id >= 0) {
			(void)publishDnaAllocationResponse(new_node_id, dnaCurrentUniqueId, dnaCurrentUniqueIdLen);
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

int8_t CAN::allocateNode() {
	// dronecan reserves 126 and 127
	static constexpr uint8_t MAX_DYNAMIC_NODE_ID = 125;

	// try preffered if first
	if (dnaPreferredNodeId != UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ANY_NODE_ID &&
		dnaPreferredNodeId >= CANARD_MIN_NODE_ID &&
		dnaPreferredNodeId <= MAX_DYNAMIC_NODE_ID &&
		canNodes[dnaPreferredNodeId].status.mode == UAVCAN_PROTOCOL_NODESTATUS_MODE_OFFLINE) {
		return dnaPreferredNodeId;
	}

	for (int currId = nextAvailableID; currId <= MAX_DYNAMIC_NODE_ID; currId++) {
		if (canNodes[currId].status.mode == UAVCAN_PROTOCOL_NODESTATUS_MODE_OFFLINE) {
			nextAvailableID = currId + 1;
			return currId;
		}
	}

	return -1; // no IDs available
}

DnaStage CAN::detectDnaRequestStage(const uavcan_protocol_dynamic_node_id_Allocation& msg) const {

	constexpr uint8_t MAX_LEN = UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_LENGTH_OF_UNIQUE_ID_IN_REQUEST;
	constexpr uint8_t STAGE3_LEN = UAVCAN_UNIQUE_ID_LENGTH - MAX_LEN * 2U;

	const uint8_t len = msg.unique_id.len;

	// len should be 6 or 4
	if (len != MAX_LEN && len != STAGE3_LEN) {
		return DnaStage::INVALID;
	}

	if (msg.first_part_of_unique_id) {
		return DnaStage::STAGE_1;
	}

	if (len == MAX_LEN) {
		return DnaStage::STAGE_2;
	}

	if (len == STAGE3_LEN) {
		return DnaStage::STAGE_3;
	}

	return DnaStage::INVALID;
}

DnaStage CAN::getExpectedDnaStage() const {
	constexpr uint8_t MAX_LEN = UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_LENGTH_OF_UNIQUE_ID_IN_REQUEST;

	switch (dnaCurrentUniqueIdLen) {
		case 0: 
			return DnaStage::STAGE_1;
		case MAX_LEN:
			return DnaStage::STAGE_2;
		case MAX_LEN * 2:
			return DnaStage::STAGE_3;
		default:
			return DnaStage::INVALID;
	}
}


void CAN::resetDnaInProgress() {
	dnaCurrentUniqueIdLen = 0;
	dnaPreferredNodeId = UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ANY_NODE_ID;
	dnaLastAcceptedTick = 0;
}

int16_t CAN::publishDnaAllocationResponse(uint8_t node_id, const uint8_t* unique_id, uint8_t unique_id_len) {
	uavcan_protocol_dynamic_node_id_Allocation msg {};
	msg.node_id = node_id;
	msg.first_part_of_unique_id = false;
	msg.unique_id.len = unique_id_len;
	memcpy(msg.unique_id.data, unique_id, unique_id_len);

	uint8_t buffer[UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_SIZE];
	uint32_t len = uavcan_protocol_dynamic_node_id_Allocation_encode(&msg, buffer);

	return broadcast(
		CanardTransferTypeBroadcast,
		UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_SIGNATURE,
		UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID,
		&transfer_id,
		CANARD_TRANSFER_PRIORITY_LOW,
		buffer,
		len
	);
}

uint32_t getFDCANDLC(uint8_t len) {
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
        default: return FDCAN_DLC_BYTES_8; // fallback (or assert)
    }
}

/*
Function to convert all canard CAN frames and send them through HAL

Consider removing for loop
*/
void CAN::sendCANTx() {
	CanardCANFrame* frame = canardPeekTxQueue(&canard);
	if (frame == nullptr) return;

	if (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) > 0) {
		FDCAN_TxHeaderTypeDef txHeader;
		txHeader.Identifier = frame->id & CANARD_CAN_EXT_ID_MASK;
		txHeader.IdType = FDCAN_EXTENDED_ID;
		txHeader.TxFrameType = FDCAN_DATA_FRAME;
		txHeader.DataLength = getFDCANDLC(frame->data_len);
		txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
		txHeader.BitRateSwitch = FDCAN_BRS_OFF;
		txHeader.FDFormat = FDCAN_CLASSIC_CAN;
		txHeader.TxEventFifoControl = FDCAN_STORE_TX_EVENTS;
		txHeader.MessageMarker = 3;

		uint8_t txData[8];
		memcpy(txData, frame->data, frame->data_len);

		bool success = HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &txHeader, txData) == HAL_OK;

		if (success) {
			canardPopTxQueue(&canard);
		}
	}
}

bool CAN::routineTasks() {

	sendCANTx();

	uint32_t tick = HAL_GetTick();

	if (tick > last1HzTick + UAVCAN_PROTOCOL_NODESTATUS_MAX_BROADCASTING_PERIOD_MS/2) {
		last1HzTick = tick;
		process1HzTasks();
	}

	return true;
}

void CAN::sendNodeStatus() {
	uint8_t buffer[UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_MAX_SIZE];

    nodeStatus.uptime_sec = HAL_GetTick()/1000LL;
    nodeStatus.health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
    nodeStatus.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;
    nodeStatus.sub_mode = 0;
    // put whatever you like in here for display in GUI
    nodeStatus.vendor_specific_status_code = 1234;

    uint32_t len = uavcan_protocol_NodeStatus_encode(&nodeStatus, buffer);

    // we need a static variable for the transfer ID. This is
    // incremeneted on each transfer, allowing for detection of packet
    // loss

    broadcast(CanardTransferTypeBroadcast,
			UAVCAN_PROTOCOL_NODESTATUS_SIGNATURE,
			UAVCAN_PROTOCOL_NODESTATUS_ID,
			&transfer_id,
			CANARD_TRANSFER_PRIORITY_LOW,
			buffer,
			len
	);

}

void CAN::process1HzTasks() {

	uint32_t timestamp_msec = HAL_GetTick();

	// Check if nodes invalid
	for (int i = CANARD_MIN_NODE_ID; i <= CANARD_MAX_NODE_ID; i++) {
		// Make copy of status in case it changes

		if (timestamp_msec-canNodes[i].lastSeenTick > UAVCAN_PROTOCOL_NODESTATUS_OFFLINE_TIMEOUT_MS) {
			canNodes[i].status.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_OFFLINE;
		}
	}

	// Transmit NodeStatus
	sendNodeStatus();
}


/*
Wrapper function with mutex
*/
int16_t CAN::broadcastObj(CanardTxTransfer* transfer) {
//	osStatus_t status = osMutexAcquire(canBroadcastMutex, CAN_BROADCAST_MUTEX_TIMEOUT);

//	if (status != osOK){
//		return -1; // handle failure
//	}

	int16_t res = canardBroadcastObj(&canard, transfer);
//	osMutexRelease(canBroadcastMutex);

	return res;
}

int16_t CAN::broadcast(
	CanardTransferType transfer_type,
	uint64_t data_type_signature,
	uint16_t data_type_id,
	uint8_t* inout_transfer_id,
	uint8_t priority,
	const uint8_t* payload,
	uint16_t payload_len
	#if CANARD_ENABLE_CANFD
		, bool canfd
	#endif
	#if CANARD_ENABLE_DEADLINE
		, uint64_t deadline_usec
	#endif
	#if CANARD_MULTI_IFACE
		, uint8_t iface_mask
	#endif
	#if CANARD_ENABLE_TAO_OPTION
		, bool tao
	#endif
)
{
	CanardTxTransfer transfer_object;
	transfer_object.transfer_type = transfer_type;
	transfer_object.data_type_signature = data_type_signature;
	transfer_object.data_type_id = data_type_id;
	transfer_object.inout_transfer_id = inout_transfer_id;
	transfer_object.priority = priority;
	transfer_object.payload = payload;
	transfer_object.payload_len = payload_len;
	
	#if CANARD_ENABLE_CANFD
		transfer_object.canfd = canfd;
	#endif
	#if CANARD_ENABLE_DEADLINE
		transfer_object.deadline_usec = deadline_usec;
	#endif
	#if CANARD_MULTI_IFACE
		transfer_object.iface_mask = iface_mask;
	#endif
	#if CANARD_ENABLE_TAO_OPTION
		transfer_object.tao = tao;
	#endif

	return broadcastObj(&transfer_object);
}