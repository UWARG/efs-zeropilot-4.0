#include "power_module.hpp"


PowerModule::PowerModule(I2C_HandleTypeDef* hi2c) : hi2c(hi2c) {
}

bool PowerModule::init() {
    callbackCount = 0;
    bool success = (HAL_I2C_IsDeviceReady(hi2c, INA228_ADDR << 1, 1, 100) == HAL_OK);
    uint8_t pData[2];

    /*
    Configuring the registers
    */
    pData[0] = (SHUNT_CAL_VALUE >> 8) & 0x7F;
    pData[1] = SHUNT_CAL_VALUE & 0xFF;
    success &= (HAL_I2C_Mem_Write(hi2c, INA228_ADDR << 1, REG_SHUNT_CAL.address, I2C_MEMADD_SIZE_8BIT, pData, REG_SHUNT_CAL.byte_size, 100) == HAL_OK);


    pData[0] = (ADC_CONFIG_VALUE >> 8) & 0xFF;
    pData[1] = ADC_CONFIG_VALUE & 0xFF;
    success &= (HAL_I2C_Mem_Write(hi2c, INA228_ADDR << 1, REG_ADC_CONFIG.address, I2C_MEMADD_SIZE_8BIT, pData, REG_ADC_CONFIG.byte_size, 100) == HAL_OK);

    uint16_t config = CONFIG_VALUE;
    pData[0] = (config >> 8) & 0xFF;
    pData[1] = config & 0xFF;
    success &= (HAL_I2C_Mem_Write(hi2c, INA228_ADDR << 1, REG_CONFIG.address, I2C_MEMADD_SIZE_8BIT, pData, REG_CONFIG.byte_size, 100) == HAL_OK);


    /*
    Starting DMA loop
    */

   if(success) {
        dataFilled = 0;
        parse(hi2c);
    }

    return success;
}


bool PowerModule::writeRegister(
                                uint16_t memAddress,
                                uint8_t * pData,
                                uint16_t size,
                                I2C_HandleTypeDef *hi2c) {

    return HAL_I2C_Mem_Write_DMA(hi2c, INA228_ADDR << 1, memAddress, I2C_MEMADD_SIZE_8BIT, pData, size) == HAL_OK;

}

bool PowerModule::readRegister(
                                uint16_t memAddress,
                                uint8_t * pData,
                                uint16_t size,
                                I2C_HandleTypeDef *hi2c) {

    return HAL_I2C_Mem_Read_DMA(hi2c, INA228_ADDR << 1, memAddress, I2C_MEMADD_SIZE_8BIT, pData, size) == HAL_OK;
}

void PowerModule::I2C_MemRxCpltCallback() {
    callbackCount++;

    switch(callbackCount) {
        case 1: // read current
            readRegister(REG_CURRENT.address, currentData, REG_CURRENT.byte_size, hi2c);
            break;
        case 2: // read power
            readRegister(REG_POWER.address, powerData, REG_POWER.byte_size, hi2c);
            break;
        case 3: // read charge
            readRegister(REG_CHARGE.address, chargeData, REG_CHARGE.byte_size, hi2c);
            break;
        case 4: // read energy
            readRegister(REG_ENERGY.address, energyData, REG_ENERGY.byte_size, hi2c);
            break;
        case 5:
            callbackCount = 0;
            dataFilled = 1;
            break;
        default:
            break;
    }

    return;
}



void PowerModule::parse(I2C_HandleTypeDef *hi2c) {
    // start the cycle
    if (dataFilled) return;
    readRegister(REG_VBUS.address, vbusData, REG_VBUS.byte_size, hi2c);
}

bool PowerModule::readData(PMData_t *data) {

    // two's complement
    processedData.busVoltage = (((vbusData[0] << 16) | (vbusData[1] << 8) | vbusData[2]) >> 4) * VBUS_LSB;
    
    processedData.current = (((currentData[0] << 16) | (currentData[1] << 8) | currentData[2]) >> 4) * CURRENT_LSB;
    
    processedData.charge = ((((uint64_t)chargeData[0] << 32) | (chargeData[1] << 24) |
                             (chargeData[2] << 16) | (chargeData[3] << 8) | chargeData[4])) * CHARGE_LSB;

    // unsigned
    processedData.power = ((powerData[0] << 16) | (powerData[1] << 8) | powerData[2]) * POWER_LSB;

    processedData.energy = ((((uint64_t)energyData[0] << 32) | (energyData[1] << 24) |
                             (energyData[2] << 16) | (energyData[3] << 8) | energyData[4])) * ENERGY_LSB;

    *data = processedData;

    dataFilled = 0;
    parse(hi2c);

    return true;
}

I2C_HandleTypeDef* PowerModule::getI2C() {
    return hi2c;
}


