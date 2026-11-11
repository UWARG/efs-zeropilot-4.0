#include "zp_bit.hpp"
#include <utility>

namespace ZP_BIT {

    namespace {

        typedef struct {
            BitState_e currentState; // Current debounced state
            BitState_e latchedFault; // Latched on fault for critical BITs
            BitState_e liveState; // Current running state
            uint32_t edgeMs; // Timestamp of last change of running state
            uint32_t failMs;
            uint32_t clearMs;
            ZP_Error lastError; // Last reported error
        } BitStatus_t;

        BitStatus_t bitStatus[static_cast<uint16_t>(ZP_BIT_ID::NUM_BIT_IDS)];
        ISystemUtils* clockDriver = nullptr;


        inline bool indexValid(ZP_BIT_ID id);
    }

    ZP_Error init(ISystemUtils* clock) {
        if (clock == nullptr) {
            return ZP_ERROR_NULLPTR;
        }

        clockDriver = clock;

        for (uint16_t i = 0; i < static_cast<uint16_t>(ZP_BIT_ID::NUM_BIT_IDS); i++) {
            bitStatus[i].currentState = BitState_e::UNKNOWN;
            bitStatus[i].latchedFault = BitState_e::UNKNOWN;
            bitStatus[i].liveState = BitState_e::UNKNOWN;
            bitStatus[i].edgeMs = 0;
            bitStatus[i].failMs = BIT_CONFIG[i].failMs;
            bitStatus[i].clearMs = BIT_CONFIG[i].clearMs;
            bitStatus[i].lastError = ZP_Error();
        }

        return ZP_ERROR_OK;
    }

    ZP_Error report(ZP_BIT_ID id, ZP_Error status) {
        if (!indexValid(id)) {
            return ZP_ERROR_RANGE;
        }
        if (clockDriver == nullptr) {
            return ZP_ERROR_NOT_READY;
        }

        const BitConfig_t& config = BIT_CONFIG[static_cast<uint16_t>(id)];
        BitStatus_t& state = bitStatus[static_cast<uint16_t>(id)];

        const BitState_e OBSERVED = (status == ZP_ERROR_OK) ? BitState_e::SUCCESS : BitState_e::FAILURE;
        const uint32_t NOW = clockDriver->getCurrentTimestampMs();

        state.lastError = std::move(status);

        // A change of run restarts the debounce window
        if (state.liveState != OBSERVED) {
            state.liveState = OBSERVED;
            state.edgeMs = NOW;
        }

        const uint32_t ELAPSED_MS = NOW - state.edgeMs;

        BitState_e newState = state.currentState;
        if (OBSERVED == BitState_e::SUCCESS) {
            if (state.currentState == BitState_e::UNKNOWN || ELAPSED_MS >= state.clearMs) {
                newState = BitState_e::SUCCESS;
            }
        } else if (ELAPSED_MS >= state.failMs) {
            newState = BitState_e::FAILURE;
        }

        if (newState != state.currentState) {
            state.currentState = newState;

            if (newState == BitState_e::FAILURE && config.level == BitLevel_e::CRITICAL) {
                state.latchedFault = BitState_e::FAILURE;
            }
        }

        return ZP_ERROR_OK;
    }



    ZP_Error clearLatched() {
        for (uint16_t i = 0; i < static_cast<uint16_t>(ZP_BIT_ID::NUM_BIT_IDS); i++) {
            bitStatus[i].latchedFault = bitStatus[i].currentState;
        }

        return ZP_ERROR_OK;
    }

    ZP_Error setFailPersistence(ZP_BIT_ID id, uint32_t failMs) {
        if (!indexValid(id)) {
            return ZP_ERROR_RANGE;
        }

        bitStatus[static_cast<uint16_t>(id)].failMs = failMs;
        return ZP_ERROR_OK;
    }

    ZP_Error setClearPersistence(ZP_BIT_ID id, uint32_t clearMs) {
        if (!indexValid(id)) {
            return ZP_ERROR_RANGE;
        }

        bitStatus[static_cast<uint16_t>(id)].clearMs = clearMs;
        return ZP_ERROR_OK;
    }

    ZP_Error setPersistence(ZP_BIT_ID id, uint32_t failMs, uint32_t clearMs) {
        ZP_Error result = ZP_ERROR_OK;
        result |= setFailPersistence(id, failMs);
        result |= setClearPersistence(id, clearMs);
        return result;
    }

    ZP_Error getLive(ZP_BIT_ID id, BitState_e& outState) {
        if (!indexValid(id)) {
            outState = BitState_e::UNKNOWN;
            return ZP_ERROR_RANGE;
        }

        outState = bitStatus[static_cast<uint16_t>(id)].currentState;
        return ZP_ERROR_OK;
    }

    ZP_Error getLatched(ZP_BIT_ID id, BitState_e& outState) {
        if (!indexValid(id)) {
            outState = BitState_e::UNKNOWN;
            return ZP_ERROR_RANGE;
        }

        outState = bitStatus[static_cast<uint16_t>(id)].latchedFault;
        return ZP_ERROR_OK;
    }

    ZP_Error getError(ZP_BIT_ID id, ZP_Error& outError) {
        if (!indexValid(id)) {
            return ZP_ERROR_RANGE;
        }

        outError = ZP_Error(bitStatus[static_cast<uint16_t>(id)].lastError);
        return ZP_ERROR_OK;
    }



    ZP_Error name(ZP_BIT_ID id, const char*& outName) {
        if (!indexValid(id)) {
            return ZP_ERROR_RANGE;
        }

        outName = BIT_CONFIG[static_cast<uint16_t>(id)].name;
        return ZP_ERROR_OK;
    }

     namespace {

        inline bool indexValid(ZP_BIT_ID id) {
            return id < ZP_BIT_ID::NUM_BIT_IDS;
        }
    }
}
