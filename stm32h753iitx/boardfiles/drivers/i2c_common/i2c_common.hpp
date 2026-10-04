// I2C Common Functions
#include "stm32h7xx_hal.h"
#include <cstdint>
class I2cCommon {
public:
    //readRegisterBlocking
    /* Reads data from the I2C device in blocking mode. Returns true if the read 
       succeeded, false on error, busy, or timeout.*/
    
    /* Multi-byte read. Takes the register address to read from, a pointer to the
       destination buffer, the number of bytes to read, and a timeout in ms. 
       The buffer must hold at least `size` bytes.*/
    bool readBlocking(uint16_t memAddress, uint8_t* pData, uint16_t size, uint32_t timeout = HAL_MAX_DELAY);
    /* Single-byte read. Takes the register address to read from, a reference to
       the variable that receives the byte, and a timeout in ms.*/
    bool readBlocking(uint16_t memAddress, uint8_t& out, uint32_t timeout = HAL_MAX_DELAY);


    //writeRegisterBlocking
    /* Writes data to the I2C device in blocking mode. Returns true if the write 
       succeeded, false on error, busy, or timeout.*/

    /* Multi-byte write. Takes the register address to write to, a pointer to the
       source buffer, the number of bytes to write, and a timeout in ms.
       The buffer must hold at least `size` bytes.*/
    bool writeBlocking(uint16_t memAddress, const uint8_t* pData, uint16_t size, uint32_t timeout = HAL_MAX_DELAY);
    /* Single-byte write. Takes the register address to write to, the byte value
       to write, and a timeout in ms.*/
    bool writeBlocking(uint16_t memAddress, uint8_t value, uint32_t timeout = HAL_MAX_DELAY);    
    
    
    //getI2C


    //readDMA (formerly readRegister)
    /*This function reads data from the I2C device using DMA. It takes
      the memory address to read from, a pointer to the data buffer, 
      and the size of the data to read. It returns true if the read 
      operation was successful, false otherwise.*/
    bool readDMA(uint16_t MemAddress, uint8_t * pData, uint16_t Size);


    //writeDMA (formerly writeRegister)
    /*This function writes data to the I2C device using DMA. It takes 
      the memory address to write to, a pointer to the data buffer, and 
      the size of the data to write. It returns true if the write operation 
      was successful, false otherwise.*/
    bool writeDMA(uint16_t MemAddress, uint8_t * pData, uint16_t Size);


    //readPolling (TBD)


    //writePolling (TBD)


    //readInterrupt (TBD)


    //writeInterrupt (TBD)
};
