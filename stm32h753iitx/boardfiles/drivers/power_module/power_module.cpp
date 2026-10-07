#include "power_module.hpp"


PowerModule::PowerModule(I2C_HandleTypeDef* hi2c) : i2cCommon(hi2c,INA228_ADDR) {}

I2C_HandleTypeDef* PowerModule::getI2C() {
    return i2cCommon.getI2C();
}

bool PowerModule::init() {
    callbackCount = 0;
    bool success = i2cCommon.isDeviceReady();
    if (!success) return false;

    uint8_t pData[2]; // tmp buffer for writing to registers

    // Perform a soft reset of the INA228 by writing to the configuration register
    pData[0] = (CONFIG_RESET >> 8) & 0xFF;  
    pData[1] = CONFIG_RESET & 0xFF;         
    success &= i2cCommon.writePolling(REG_CONFIG.address, pData, REG_CONFIG.byte_size, 100);

    HAL_Delay(5); // Give the chip a few milliseconds to process the reset

    // Set the actual operating configuration
    pData[0] = (CONFIG_VALUE >> 8) & 0xFF;      
    pData[1] = CONFIG_VALUE & 0xFF;         
    success &= i2cCommon.writePolling(REG_CONFIG.address, pData, REG_CONFIG.byte_size, 100);

    // Write the ADC configuration for continuous reading and 16 samples averaged
    pData[0] = (ADC_CONFIG_VALUE >> 8) & 0xFF;  
    pData[1] = ADC_CONFIG_VALUE & 0xFF;         
    success &= i2cCommon.writePolling(REG_ADC_CONFIG.address, pData, REG_ADC_CONFIG.byte_size, 100);

    // Write the shunt calibration value to the appropriate register
    pData[0] = (SHUNT_CAL_VALUE >> 8) & 0x7F;  
    pData[1] = SHUNT_CAL_VALUE & 0xFF;     
    success &= i2cCommon.writePolling(REG_SHUNT_CAL.address, pData, REG_SHUNT_CAL.byte_size, 100);

    // Start the DMA loop
    if (success) {
        dataFilled = 0;
        parse();
    }

    return success;
}

void PowerModule::I2C_MemRxCpltCallback() {
    callbackCount++;
    bool success = true;

    switch(callbackCount) {
        case 1: // read current
            success = i2cCommon.readDMA(REG_CURRENT.address, currentData, REG_CURRENT.byte_size);
            break;
        case 2: // read power
            success = i2cCommon.readDMA(REG_POWER.address, powerData, REG_POWER.byte_size);
            break;
        case 3: // read charge
            success = i2cCommon.readDMA(REG_CHARGE.address, chargeData, REG_CHARGE.byte_size);
            break;
        case 4: // read energy
            success = i2cCommon.readDMA(REG_ENERGY.address, energyData, REG_ENERGY.byte_size);
            break;
        case 5: // read die temperature
            success = i2cCommon.readDMA(REG_DIETEMP.address, dietempData, REG_DIETEMP.byte_size);
            break;
        case 6:
            callbackCount = 0;
            dataFilled = 1;
            break;
        default:
            break;
    }

    // If one read failed to start, reset the chain, next readData() call restarts from VBUS
    if (!success) {
        callbackCount = 0;
    }

    return;
}

void PowerModule::I2C_ErrorCallback() {
    // HAL aborts the transfer before invoking this callback, so only the
    // chain state needs resetting, the next readData() call restarts the cycle
    callbackCount = 0;
}

void PowerModule::parse() {
    if (dataFilled) return;
    
    // Start the cycle
    callbackCount = 0;
    i2cCommon.readDMA(REG_VBUS.address, vbusData, REG_VBUS.byte_size);
}

bool PowerModule::readData(PMData_t *data) {
    // No fresh sample set yet, kick off the cycle or restarting it if it stalled, and report no fresh data
    if (!dataFilled) {
        parse();
        return false;
    }

    // Parse VBUS from the raw data, which is a 24-bit unsigned value.
    // No sign extension or right-shift needed, since VBUS is unsigned.
    processedData.busVoltage = (((vbusData[0] << 16) | (vbusData[1] << 8) | vbusData[2]) >> 4) * VBUS_LSB;

    // Parse current from the raw data, which is a 24-bit signed value.
    // Perform sign extension and right-shift to get the correct value.
    int32_t raw_current = (currentData[0] << 24) | (currentData[1] << 16) | (currentData[2] << 8);
    raw_current /= 256; // Arithmetic shift down to 24-bit, preserving sign
    processedData.current = (raw_current >> 4) * CURRENT_LSB;

    // Parse charge from the raw data, which is a 40-bit signed value.
    // Pack the 5 bytes into a 64-bit integer and perform sign extension if necessary.
    int64_t raw_charge = (((uint64_t)chargeData[0] << 32) | 
                          ((uint64_t)chargeData[1] << 24) |
                          ((uint64_t)chargeData[2] << 16) | 
                          ((uint64_t)chargeData[3] << 8) | 
                           (uint64_t)chargeData[4]);
    if (raw_charge & (1ULL << 39)) {
        raw_charge |= 0xFFFFFF0000000000ULL; 
    }
    processedData.charge = raw_charge * CHARGE_LSB;

    // Parse power from the raw data, which is a 24-bit unsigned value.
    // No sign extension or right-shift needed, since power is unsigned.
    processedData.power = ((powerData[0] << 16) | (powerData[1] << 8) | powerData[2]) * POWER_LSB;

    // Parse energy from the raw data, which is a 40-bit unsigned value.
    // Pack the 5 bytes into a 64-bit integer. No sign extension needed since energy is unsigned.
    processedData.energy = (((uint64_t)energyData[0] << 32) | 
                            ((uint64_t)energyData[1] << 24) |
                            ((uint64_t)energyData[2] << 16) | 
                            ((uint64_t)energyData[3] << 8) | 
                             (uint64_t)energyData[4]) * ENERGY_LSB;

    // Parse die temperature from the raw data, which is a 16-bit signed value.
    int16_t raw_dietemp = (dietempData[0] << 8) | dietempData[1];
    processedData.temperature = raw_dietemp * DIETEMP_LSB;

    *data = processedData; // Copy the processed data to the output parameter

    // Reset dataFilled flag and restart the parsing cycle
    dataFilled = 0;
    parse();

    return true;
}

I2C_HandleTypeDef* PowerModule::getI2C() {
    return i2cCommon.getI2C();
}
