#include "i2c_common.hpp"

I2cCommon::I2cCommon(I2C_HandleTypeDef* hi2c, uint8_t addr7)
    : hi2c_(hi2c), 
    addr_(addr7 << 1) {}       /* Initializes the I2cCommon class with the I2C handle and the device's
                                7-bit address. Pass the plain 7-bit address (do NOT pre-shift it); the
                                constructor shifts it left by 1 for HAL. Both are saved for use in
                                subsequent I2C operations.*/


//getI2C
I2C_HandleTypeDef* I2cCommon::getI2C() const {
    return hi2c_;
}


//isDeviceReady
bool I2cCommon::isDeviceReady(uint32_t trials, uint32_t timeout) {
    return HAL_I2C_IsDeviceReady(hi2c_, addr_, trials, timeout) == HAL_OK;
}


//Polling
//multi-byte read
bool I2cCommon::readPolling(uint16_t memAddress, uint8_t* pData, uint16_t size, uint32_t timeout) {
    return HAL_I2C_Mem_Read(hi2c_, addr_, memAddress, I2C_MEMADD_SIZE_8BIT, pData, size, timeout) == HAL_OK;
}
//single-byte read
bool I2cCommon::readPolling(uint16_t memAddress, uint8_t& out, uint32_t timeout) {
    return readPolling(memAddress, &out, 1, timeout); 
}

//multi-byte write
bool I2cCommon::writePolling(uint16_t memAddress, const uint8_t* pData, uint16_t size, uint32_t timeout) {
    // HAL takes a non-const pointer but only reads from it when transmitting
    return HAL_I2C_Mem_Write(hi2c_, addr_, memAddress, I2C_MEMADD_SIZE_8BIT,
                             const_cast<uint8_t*>(pData), size, timeout) == HAL_OK;
}
//single-byte write
bool I2cCommon::writePolling(uint16_t memAddress, uint8_t value, uint32_t timeout) {
    return writePolling(memAddress, &value, 1, timeout);
}



//DMA
//readDMA
bool I2cCommon::readDMA(uint16_t memAddress, uint8_t* pData, uint16_t size) {
    return HAL_I2C_Mem_Read_DMA(hi2c_, addr_, memAddress, I2C_MEMADD_SIZE_8BIT, pData, size) == HAL_OK;
}
//writeDMA
bool I2cCommon::writeDMA(uint16_t memAddress, uint8_t* pData, uint16_t size) {
    return HAL_I2C_Mem_Write_DMA(hi2c_, addr_, memAddress, I2C_MEMADD_SIZE_8BIT, pData, size) == HAL_OK;
}




//Raw (non-register) transfers
//transmitPolling
bool I2cCommon::transmitPolling(const uint8_t* pData, uint16_t size, uint32_t timeout) {
    // HAL takes a non-const pointer but only reads from it when transmitting
    return HAL_I2C_Master_Transmit(hi2c_, addr_, const_cast<uint8_t*>(pData), size, timeout) == HAL_OK;
}
//receivePolling
bool I2cCommon::receivePolling(uint8_t* pData, uint16_t size, uint32_t timeout) {
    return HAL_I2C_Master_Receive(hi2c_, addr_, pData, size, timeout) == HAL_OK;
}

//transmitInterrupt
bool I2cCommon::transmitInterrupt(const uint8_t* pData, uint16_t size) {
    return HAL_I2C_Master_Transmit_IT(hi2c_, addr_, const_cast<uint8_t*>(pData), size) == HAL_OK;
}
//receiveInterrupt
bool I2cCommon::receiveInterrupt(uint8_t* pData, uint16_t size) {
    return HAL_I2C_Master_Receive_IT(hi2c_, addr_, pData, size) == HAL_OK;
}
