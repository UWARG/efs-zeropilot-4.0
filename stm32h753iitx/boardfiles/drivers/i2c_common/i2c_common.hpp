#pragma once

// I2C Common Functions
#include "stm32h7xx_hal.h"
#include <cstdint>


class I2cCommon {
public:
    //Constructor
    /*This constructor initializes the I2cCommon class with the provided I2C 
      handle and the 7-bit address of the I2C device (DO NOT PRE-SHIFT IT). The 
      constructor shifts the address left by 1 for HAL and saves both 
      parameters for use in subsequent I2C operations.*/
    I2cCommon(I2C_HandleTypeDef* hi2c, uint8_t addr7);

    //getI2C
    /* Returns the I2C handle this device is on.*/
    I2C_HandleTypeDef* getI2C() const;


    //isDeviceReady
    /* Checks whether the device acknowledges its address on the bus. Takes the
       number of attempts and a timeout in ms. Returns true if the device
       responded, false if it did not (not connected, wrong address, bus fault,
       or timeout).*/
    bool isDeviceReady(uint32_t trials = READY_TRIALS_DEFAULT, uint32_t timeout = READY_TIMEOUT_MS_DEFAULT);    

    //readRegisterPolling (formerly readRegisterBlocking)
    /* Reads data from the I2C device in polling mode. Returns true if the read 
       succeeded, false on error, busy, or timeout.*/
    
    /* Multi-byte read. Takes the register address to read from, a pointer to the
       destination buffer, the number of bytes to read, and a timeout in ms. 
       The buffer must hold at least `size` bytes.*/
    bool readPolling(uint16_t memAddress, uint8_t* pData, uint16_t size, uint32_t timeout = HAL_MAX_DELAY);
    /* Single-byte read. Takes the register address to read from, a reference to
       the variable that receives the byte, and a timeout in ms.*/
    bool readPolling(uint16_t memAddress, uint8_t& out, uint32_t timeout = HAL_MAX_DELAY);


    //writeRegisterPolling (formerly writeRegisterBlocking)
    /* Writes data to the I2C device in polling mode. Returns true if the write 
       succeeded, false on error, busy, or timeout.*/

    /* Multi-byte write. Takes the register address to write to, a pointer to the
       source buffer, the number of bytes to write, and a timeout in ms.
       The buffer must hold at least `size` bytes.*/
    bool writePolling(uint16_t memAddress, const uint8_t* pData, uint16_t size, uint32_t timeout = HAL_MAX_DELAY);
    /* Single-byte write. Takes the register address to write to, the byte value
       to write, and a timeout in ms.*/
    bool writePolling(uint16_t memAddress, uint8_t value, uint32_t timeout = HAL_MAX_DELAY);    
    

    //readDMA (formerly readRegister)
    /*This function reads data from the I2C device using DMA. It takes
      the memory address to read from, a pointer to the data buffer, 
      and the size of the data to read. It returns true if the read 
      operation was successful, false otherwise.*/
    bool readDMA(uint16_t memAddress, uint8_t * pData, uint16_t size);


    //writeDMA (formerly writeRegister)
    /*This function writes data to the I2C device using DMA. It takes 
      the memory address to write to, a pointer to the data buffer, and 
      the size of the data to write. It returns true if the write operation 
      was successful, false otherwise.*/
    bool writeDMA(uint16_t memAddress, uint8_t * pData, uint16_t size);


    //readInterrupt (TBD)


    //writeInterrupt (TBD)

    //transmitPolling / receivePolling
    /* Raw (non-register) transfers in polling mode, for devices that use
       command/response frames instead of a register map (e.g. TF02-Pro).
       Returns true if the transfer succeeded, false on error, busy, or 
       timeout.*/

    /* Transmit. Takes a pointer to the source buffer, the number of bytes to
       send, and a timeout in ms. The buffer must hold at least `size` bytes.*/
    bool transmitPolling(const uint8_t* pData, uint16_t size, uint32_t timeout = HAL_MAX_DELAY);
    /* Receive. Takes a pointer to the destination buffer, the number of bytes
       to read, and a timeout in ms. The buffer must hold at least `size` bytes.*/
    bool receivePolling(uint8_t* pData, uint16_t size, uint32_t timeout = HAL_MAX_DELAY);


    //transmitInterrupt / receiveInterrupt
    /* Raw (non-register) transfers in interrupt mode. Returns immediately: true
       means the transfer was started, false if HAL rejected it (e.g. the bus is
       busy).*/

    /* Transmit. Takes a pointer to the source buffer and the number of bytes to
       send. The buffer must stay valid until the transfer completes, so pass
       a member or a static, never a local variable.*/
    bool transmitInterrupt(const uint8_t* pData, uint16_t size);
    /* Receive. Takes a pointer to the destination buffer and the number of
       bytes to read. The buffer must stay valid until the transfer completes
       and hold at least `size` bytes.*/
    bool receiveInterrupt(uint8_t* pData, uint16_t size);

private:    
    //default constants
    static constexpr uint32_t READY_TRIALS_DEFAULT = 1; // Default number of trials for isDeviceReady
    static constexpr uint32_t READY_TIMEOUT_MS_DEFAULT = 100; // Default timeout in ms
    
    //data saved by constructor
    //Pointer to the I2C handle
    I2C_HandleTypeDef* hi2c_;
    //7-bit address of the I2C device (shifted left by 1 for HAL)
    uint16_t addr_;
};
