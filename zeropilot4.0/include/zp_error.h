#pragma once

typedef enum {
    ZP_ERROR_OK                   = 0x0000, // Operation completed successfully
    ZP_ERROR_FAIL                 = 0x0001, // Generic failure. Prioritize classifying errors listed below first.
    ZP_ERROR_TIMEOUT              = 0x0002, // Operation timed out
    ZP_ERROR_BUSY                 = 0x0004, // Resource or peripheral is busy
    ZP_ERROR_INVALID_PARAM        = 0x0008, // Provided input is invalid
    ZP_ERROR_NULLPTR              = 0x0010, // Null pointer dereference attempt
    ZP_ERROR_UNSUPPORTED          = 0x0020, // Feature or operation not supported
    ZP_ERROR_NOT_READY            = 0x0040, // Resource is not initialized or ready
    ZP_ERROR_ALREADY_INITIALIZED  = 0x0080, // Resource has already been initialized
    ZP_ERROR_OUT_OF_MEMORY        = 0x0100, // Allocation or buffer failure due to memory depletion
    ZP_ERROR_RESOURCE_UNAVAILABLE = 0x0200, // Superset error for unavailable resource.
    ZP_ERROR_CRC                  = 0x0400, // CRC check failed
    ZP_ERROR_MEMORY_OVERFLOW      = 0x0800, // Buffer or FIFO queue overflow
    ZP_ERROR_PARSE                = 0x1000  // Failed to parse input
} ZP_ERROR_e;

// Overload for bitwise accumulation
inline ZP_ERROR_e& operator|=(ZP_ERROR_e& lhs, ZP_ERROR_e rhs) {
    lhs = static_cast<ZP_ERROR_e>(static_cast<uint32_t>(lhs) | static_cast<uint32_t>(rhs));
    return lhs;
}