#include "i2c_common.hpp"
#include "zp_error.h"

// Converts a HAL status into a ZP_Error, keeping the timeout/busy distinction
static ZP_Error halToZpError(HAL_StatusTypeDef status) {
    if (status == HAL_OK)      return ZP_ERROR_OK;
    if (status == HAL_TIMEOUT) return ZP_ERROR_EXT_API | ZP_ERROR_TIMEOUT;
    if (status == HAL_BUSY)    return ZP_ERROR_EXT_API | ZP_ERROR_BUSY;
    return ZP_ERROR_EXT_API | ZP_ERROR_FAIL;
}

I2cCommon::I2cCommon(I2C_HandleTypeDef* hi2c, uint8_t addr7)
    : hi2c_(hi2c),
    addr_(addr7 << 1) {}       // shift once here for HAL (see header for the full description)


//getI2C
I2C_HandleTypeDef* I2cCommon::getI2C() const {
    return hi2c_;
}


//isDeviceReady
ZP_Error I2cCommon::isDeviceReady(uint32_t trials, uint32_t timeout) {
    if (HAL_I2C_IsDeviceReady(hi2c_, addr_, trials, timeout) != HAL_OK) {
        return ZP_ERROR_EXT_API | ZP_ERROR_NACK;   // same code main's PowerModule::init returned
    }
    return ZP_ERROR_OK;
}


//Polling
//multi-byte read
ZP_Error I2cCommon::readPolling(uint16_t memAddress, uint8_t* pData, uint16_t size, uint32_t timeout) {
    return halToZpError(HAL_I2C_Mem_Read(hi2c_, addr_, memAddress, I2C_MEMADD_SIZE_8BIT, pData, size, timeout));
}
//single-byte read
ZP_Error I2cCommon::readPolling(uint16_t memAddress, uint8_t& out, uint32_t timeout) {
    return readPolling(memAddress, &out, 1, timeout);   // already a ZP_Error, return as is
}

//multi-byte write
ZP_Error I2cCommon::writePolling(uint16_t memAddress, const uint8_t* pData, uint16_t size, uint32_t timeout) {
    // HAL takes a non-const pointer but only reads from it when transmitting
    return halToZpError(HAL_I2C_Mem_Write(hi2c_, addr_, memAddress, I2C_MEMADD_SIZE_8BIT,
                                          const_cast<uint8_t*>(pData), size, timeout));
}
//single-byte write
ZP_Error I2cCommon::writePolling(uint16_t memAddress, uint8_t value, uint32_t timeout) {
    return writePolling(memAddress, &value, 1, timeout);   // already a ZP_Error, return as is
}



//DMA
//readDMA
ZP_Error I2cCommon::readDMA(uint16_t memAddress, uint8_t* pData, uint16_t size) {
    return halToZpError(HAL_I2C_Mem_Read_DMA(hi2c_, addr_, memAddress, I2C_MEMADD_SIZE_8BIT, pData, size));
}
//writeDMA
ZP_Error I2cCommon::writeDMA(uint16_t memAddress, uint8_t* pData, uint16_t size) {
    return halToZpError(HAL_I2C_Mem_Write_DMA(hi2c_, addr_, memAddress, I2C_MEMADD_SIZE_8BIT, pData, size));
}




//Raw (non-register) transfers
//transmitPolling
ZP_Error I2cCommon::transmitPolling(const uint8_t* pData, uint16_t size, uint32_t timeout) {
    // HAL takes a non-const pointer but only reads from it when transmitting
    return halToZpError(HAL_I2C_Master_Transmit(hi2c_, addr_, const_cast<uint8_t*>(pData), size, timeout));
}
//receivePolling
ZP_Error I2cCommon::receivePolling(uint8_t* pData, uint16_t size, uint32_t timeout) {
    return halToZpError(HAL_I2C_Master_Receive(hi2c_, addr_, pData, size, timeout));
}

//transmitInterrupt
ZP_Error I2cCommon::transmitInterrupt(const uint8_t* pData, uint16_t size) {
    return halToZpError(HAL_I2C_Master_Transmit_IT(hi2c_, addr_, const_cast<uint8_t*>(pData), size));
}
//receiveInterrupt
ZP_Error I2cCommon::receiveInterrupt(uint8_t* pData, uint16_t size) {
    return halToZpError(HAL_I2C_Master_Receive_IT(hi2c_, addr_, pData, size));
}