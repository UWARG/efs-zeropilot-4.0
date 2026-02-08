
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

	uint8_t first_half_unique_id[8];

 	if (transfer->source_node_id != 0) // the source node is not 0, it is not anonymous
	{
		return;
	}

    struct uavcan_protocol_dynamic_node_id_Allocation msg = {};
	uavcan_protocol_dynamic_node_id_Allocation_decode(transfer, &msg);

	if (msg.node_id != 0) // the node id is not 0, it is not anonymous
	{
		return;
	}

	memcpy(first_half_unique_id, msg.unique_id.data, 6);
	msg.unique_id.len = 6;

	// Generate the node id and allocate it
	int8_t allocated = allocateNode();
	if (allocated == -1) {
		return;
	}
	msg.node_id = allocated;

	// Send message back
	uint8_t decode_buffer[UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_SIZE];
	uint32_t encoded_size = uavcan_protocol_dynamic_node_id_Allocation_encode(&msg, decode_buffer);

	broadcast(
		CanardTransferTypeBroadcast,
		UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_SIGNATURE,
		UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID,
		&transfer_id,
		CANARD_TRANSFER_PRIORITY_LOW,
		decode_buffer,
		encoded_size
	);

}

int8_t CAN::allocateNode() {
	// check if the node id is already allocated
	int currId = nextAvailableID;

	while (canNodes[currId].status.mode == UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL) {
		currId++;
		if (currId > CANARD_MAX_NODE_ID) {
			return -1; // no more node ids available
		}
	}
	

	nextAvailableID = currId + 1;

	return currId;
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

		txHeader.Identifier = frame->id;
		txHeader.IdType = FDCAN_EXTENDED_ID;
		txHeader.TxFrameType = FDCAN_DATA_FRAME;
		txHeader.DataLength = getFDCANDLC(frame->data_len);
		txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
		txHeader.BitRateSwitch = FDCAN_BRS_OFF;
		txHeader.FDFormat = FDCAN_CLASSIC_CAN;

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