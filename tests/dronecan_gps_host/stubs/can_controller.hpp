#pragma once
#include "zp_error.h"
#include "can_listener_iface.hpp"
#include "cmsis_os2.h"
constexpr int CanardTransferTypeBroadcast=0;
struct CanardRxTransfer { uint16_t data_type_id; int transfer_type; uint8_t source_node_id; };
class CANController {
public:
    bool fail=false;
    ICanListener* listener=nullptr;
    ZP_Error subscribe(ICanListener* l, uint16_t id, uint64_t signature) {
        if(id != 1063 || signature != 0xCA41E7000F37435FULL) return ZP_ERROR_INVALID_ARG;
        listener=l; return fail ? ZP_ERROR_MEMORY_OVERFLOW : ZP_ERROR_OK;
    }
};
