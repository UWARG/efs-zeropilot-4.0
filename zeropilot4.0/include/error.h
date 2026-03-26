#pragma once

typedef enum {
    ZP_ERROR_OK,                   // Operation completed successfully
    ZP_ERROR_FAIL,                 // Generic failure. Prioritize classifying errors listed below first.
    ZP_ERROR_TIMEOUT,              // Operation timed out
    ZP_ERROR_BUSY,                 // Resource or peripheral is busy
    ZP_ERROR_INVALID_PARAM,        // Provided input is invalid
    ZP_ERROR_NULLPTR,              // Null pointer dereference attempt
    ZP_ERROR_UNSUPPORTED,          // Feature or operation not supported
    ZP_ERROR_NOT_READY,            // Resource is not initialized or ready
    ZP_ERROR_ALREADY_INITIALIZED,  // Resource has already been initialized
    ZP_ERROR_OUT_OF_MEMORY,        // Allocation or buffer failure due to memory depletion
    ZP_ERROR_RESOURCE_UNAVAILABLE, // Superset error for unavailable resource.
    ZP_ERROR_CRC,                  // CRC check failed
    ZP_ERROR_MEMORY_OVERFLOW,      // Buffer or FIFO queue overflow
    ZP_ERROR_PARSE,                // Failed to parse input
} ZP_ERROR_e;

#define ZP_RETURN_IF_ERROR(expr)       \
    do {                               \
        ZP_ERROR_e _err = (expr);     \
        if (_err != ZP_ERROR_OK) {    \
            return _err;              \
        }                              \
    } while (0)

#define ZP_CHECK_NULL(ptr)             \
    do {                               \
        if ((ptr) == NULL) {          \
            return ZP_ERROR_NULLPTR;  \
        }                              \
    } while (0)

