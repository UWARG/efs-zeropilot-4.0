
#pragma once

#include "can_iface.hpp"
#include "canard.h"
#include "stm32l5xx_hal.h"
#include "can_defines.hpp"
#include <cstdint>
#include "can_datatypes.hpp"
#include "dronecan_msgs.h"
#include "uavcan.protocol.NodeStatus.h"
#include <string.h>
#include "cmsis_os2.h"
#include "museq.hpp"



class CAN : public ICAN {

private:
	canNode canNodes[CANARD_MAX_NODE_ID + 1];
	uint8_t nextAvailableID = CANARD_MIN_NODE_ID + 1;
	FDCAN_HandleTypeDef *hfdcan;

	CanardInstance canard;

	void sendNodeStatus(); //heartbeat

	void sendCANTx();

	void handleNodeAllocation(CanardRxTransfer* transfer);
	void handleNodeStatus(CanardRxTransfer* transfer);
	
	uint8_t dlcToLength(uint32_t dlc); // data length, may delete.

	int8_t allocateNode();

    // Called once every second
	void process1HzTasks();

	uavcan_protocol_NodeStatus nodeStatus;

	uint32_t last1HzTick = 0;
	uint32_t node_id = NODE_ID;
	static uint8_t transfer_id;


public:
	CAN(FDCAN_HandleTypeDef *hfdcan);

	bool CanardShouldAcceptTransfer(const CanardInstance* ins,          ///< Library instance
		uint64_t* out_data_type_signature,  ///< Must be set by the application!
		uint16_t data_type_id,              ///< Refer to the specification
		CanardTransferType transfer_type,   ///< Refer to CanardTransferType
		uint8_t source_node_id);

	void CanardOnTransferReception(CanardInstance* ins,                 ///< Library instance
		CanardRxTransfer* transfer);

	virtual ~CAN();

	// Called as much as possible
	bool routineTasks() override;

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
			, bool canfd; ///< True if CAN FD is enabled
		#endif
		#if CANARD_ENABLE_DEADLINE
			, uint64_t deadline_usec; ///< Deadline in microseconds
		#endif
		#if CANARD_MULTI_IFACE
			, uint8_t iface_mask; ///< Bitmask of interfaces to send the transfer on
		#endif
		#if CANARD_ENABLE_TAO_OPTION
			, bool tao; ///< True if tail array optimization is enabled
		#endif
	);
};
