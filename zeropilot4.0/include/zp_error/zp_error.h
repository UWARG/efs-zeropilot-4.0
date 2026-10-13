#pragma once
#include <cstdint>

class [[nodiscard]] ZP_Error { // NOLINT
    public:
        constexpr ZP_Error() : bits(0) {}
        constexpr explicit ZP_Error(uint32_t rawBits) : bits(rawBits) {}
        constexpr ZP_Error(const ZP_Error&) = default;
        
        ZP_Error& operator=(const ZP_Error&) = delete;
        ZP_Error& operator=(ZP_Error&& rhs) {
            bits = rhs.bits;
            return *this;
        }

        ZP_Error& operator|=(const ZP_Error& rhs) {
            bits |= rhs.bits;
            return *this;
        }

        constexpr ZP_Error operator|(const ZP_Error& rhs) const { return ZP_Error(bits | rhs.bits); }
        constexpr ZP_Error operator&(const ZP_Error& rhs) const { return ZP_Error(bits & rhs.bits); }
        constexpr bool operator==(const ZP_Error& rhs) const { return bits == rhs.bits; }
        constexpr bool operator!=(const ZP_Error& rhs) const { return bits != rhs.bits; }

        constexpr uint32_t raw() const { return bits; }

    private:
        uint32_t bits;
};

constexpr ZP_Error ZP_ERROR_OK                   {0};        // Operation completed successfully
constexpr ZP_Error ZP_ERROR_FAIL                 {1u << 0};  // Generic failure. Prioritize classifying errors listed below first.
constexpr ZP_Error ZP_ERROR_UNSUPPORTED          {1u << 1};  // Feature or operation not supported
constexpr ZP_Error ZP_ERROR_CONFIG               {1u << 2};  // Device or module is misconfigured
constexpr ZP_Error ZP_ERROR_NULLPTR              {1u << 3};  // Null pointer dereference attempt
constexpr ZP_Error ZP_ERROR_INVALID_ARG          {1u << 4};  // Provided input is malformed
constexpr ZP_Error ZP_ERROR_RANGE                {1u << 5};  // Input is well-formed but outside valid bounds
constexpr ZP_Error ZP_ERROR_INVALID_DATA         {1u << 6};  // Data is structurally valid but semantically wrong
constexpr ZP_Error ZP_ERROR_PARSE                {1u << 7};  // Failed to parse input
constexpr ZP_Error ZP_ERROR_CRC                  {1u << 8};  // CRC or verification check failed
constexpr ZP_Error ZP_ERROR_NOT_READY            {1u << 9};  // Resource is not initialized or ready
constexpr ZP_Error ZP_ERROR_ALREADY_INITIALIZED  {1u << 10}; // Resource has already been initialized
constexpr ZP_Error ZP_ERROR_BUSY                 {1u << 11}; // Resource or peripheral is busy
constexpr ZP_Error ZP_ERROR_RESOURCE_UNAVAILABLE {1u << 12}; // Superset error for unavailable resource.
constexpr ZP_Error ZP_ERROR_MEMORY_OVERFLOW      {1u << 13}; // Buffer or FIFO queue overflow
constexpr ZP_Error ZP_ERROR_TIMEOUT              {1u << 14}; // Operation timed out
constexpr ZP_Error ZP_ERROR_NACK                 {1u << 15}; // Peripheral did not acknowledge
constexpr ZP_Error ZP_ERROR_EXT_API              {1u << 16}; // OR'd with a code above: failure came from HAL/RTOS/FatFs
