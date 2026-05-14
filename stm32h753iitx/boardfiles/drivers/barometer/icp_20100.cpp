#include "icp_20100.hpp"
#include "utils.h"

Barometer::Barometer(I2C_HandleTypeDef *hi2c) :
    hi2c(hi2c), callbackCount(0), fifoRegister(0) {}

static constexpr uint8_t UNLOCK_VALUE = ICP20100_MASTER_UNLOCK_KEY;
static constexpr uint8_t LOCK_VALUE   = ICP20100_MASTER_LOCK_KEY; // write 0x00 to lock
static constexpr uint32_t OTP_STATUS_POLL_TIMEOUT_MS = 1000U;
static constexpr uint32_t OTP_STATUS_POLL_INTERVAL_MS = 1U;

// OTP_STATUS2 boot status bit definitions
static constexpr uint8_t OTP_STATUS2_BOOT_STATUS_BM = 0x01U;     // Bit mask for boot status (bit 0)
static constexpr uint8_t OTP_STATUS2_BOOT_STATUS_VALID = 0x01U;  // Boot is complete
static constexpr uint8_t VERSION_B = 0xB2U;

// TRIM2_MSB register: Gain field occupies bits 4,5,6
static constexpr uint8_t ICP20100_TRIM2_MSB_GAIN_FIELD_MASK = 0x70U;  // Bits 4,5,6
static constexpr uint8_t ICP20100_TRIM2_MSB_GAIN_SHIFT = 4U;
static constexpr uint8_t ICP20100_GAIN_VALUE_MASK = 0x07U;  // 3-bit gain value from OTP

// OTP_COMMAND register field definitions
static constexpr uint8_t ICP20100_OTP_COMMAND_FIELD_MASK = 0x7FU;  // Bits 0-6
static constexpr uint8_t ICP20100_OTP_COMMAND_READ_REQUEST = 0x10U;  // Bit 4 set to request OTP read

// Register Definitions for Mikroe ICP-20100

static constexpr uint16_t ICP20100_I2C_ADDR = (0x64U << 1); // Shift by 1 for HAL
static constexpr uint8_t ICP20100_REG_MODE_SELECT = 0xC0U;
static constexpr uint8_t ICP20100_DEVICE_ID = 0x0CU;
static constexpr uint8_t ICP20100_MASTER_LOCK = 0xBEU;
static constexpr uint8_t ICP20100_OTP_CONFIG_1 = 0xACU;
static constexpr uint8_t ICP20100_OTP_STATUS = 0xB9U;
static constexpr uint8_t ICP20100_OTP_STATUS2 = 0xBFU;
static constexpr uint8_t ICP20100_VERSION_REG = 0xD3U;
static constexpr uint8_t ICP20100_OTP_DBG2 = 0xBCU;
static constexpr uint8_t ICP20100_OTP_MRA_LSB = 0xAFU;
static constexpr uint8_t ICP20100_OTP_MRA_MSB = 0xB0U;
static constexpr uint8_t ICP20100_OTP_MRB_LSB = 0xB1U;
static constexpr uint8_t ICP20100_OTP_MRB_MSB = 0xB2U;
static constexpr uint8_t ICP20100_OTP_MR_LSB = 0xADU;
static constexpr uint8_t ICP20100_OTP_MR_MSB = 0xAEU;
static constexpr uint8_t ICP20100_OTP_ADDRESS = 0xB5U;
static constexpr uint8_t ICP20100_OTP_COMMAND = 0xB6U;
static constexpr uint8_t ICP20100_OTP_RDATA = 0xB8U;
static constexpr uint8_t ICP20100_TRIM1_MSB = 0x05U;
static constexpr uint8_t ICP20100_TRIM2_LSB = 0x06U;
static constexpr uint8_t ICP20100_TRIM2_MSB = 0x07U;
static constexpr uint8_t ICP20100_FIFO_CONFIG = 0xC3U;
static constexpr uint8_t ICP20100_INTERRUPT_MASK = 0xC2U;
static constexpr uint8_t ICP20100_REG_MODE_SELECT_KEY = 0x04U;
static constexpr uint8_t ICP20100_MASTER_UNLOCK_KEY = 0x1FU;
static constexpr uint8_t ICP20100_MASTER_LOCK_KEY = 0x00U;
static constexpr uint8_t ICP20100_OTP_ENABLE_BOTH = 0x03U;
static constexpr uint8_t ICP20100_OTP_STATUS2_BOOTUP = 0x01U;
static constexpr uint8_t ICP20100_PRESS_DATA_0 = 0xFAU;
static constexpr uint8_t ICP20100_FIFO_FILL = 0xC4U;
static constexpr uint8_t ICP20100_DEVICE_STATUS = 0xCDU;
static constexpr uint8_t ICP20100_MODE_SYNC_STATUS_BIT = 0x01U;

// Forward declarations of static helper functions
static bool unlockOrLock(I2C_HandleTypeDef *hi2c, bool doLock);
static bool readRegisterBlocking(I2C_HandleTypeDef *hi2c, uint16_t memAddress, uint8_t *pData, uint16_t size, uint32_t timeout = HAL_MAX_DELAY);
static bool readRegisterBlocking(I2C_HandleTypeDef *hi2c, uint16_t memAddress, uint8_t &out, uint32_t timeout = HAL_MAX_DELAY);
static bool writeRegisterBlocking(I2C_HandleTypeDef *hi2c, uint16_t memAddress, uint8_t value, uint32_t timeout = HAL_MAX_DELAY);
static bool waitForOtpStatusClear(I2C_HandleTypeDef *hi2c);
static bool writeRegisterWithVerify(I2C_HandleTypeDef *hi2c, uint16_t memAddress, uint8_t value, uint32_t timeout = HAL_MAX_DELAY);


bool Barometer::init() {
    // Step 1: Power on ASIC

    // Step 2: Write to lock register twice to get access to main registers and initiate communication w/ I2C
    
    if (!unlockOrLock(hi2c, false)) return false;
    if (!unlockOrLock(hi2c, false)) return false;

    // Step 3: Read from the version register,
    uint8_t version = 0x00;
    if (!readRegisterBlocking(hi2c, ICP20100_VERSION_REG, version)) {
        return false;
    }

    if (version == VERSION_B) { 
        return true;
    }

	// Step 4: Check boot up status from OTP_Status2 register. Check specifically bit 0.
	uint8_t boot_status = 0x00;
	if (!readRegisterBlocking(hi2c, ICP20100_OTP_STATUS2, boot_status)) {
		return false;
	}

	// Mask boot status register to only read the 0th bit
	boot_status &= OTP_STATUS2_BOOT_STATUS_BM;

	if(boot_status == OTP_STATUS2_BOOT_STATUS_VALID){ // Initialization done, barometer did not go through power cycle.
		return true;
	}

	// Step 5: Bring ASIC into power mode to get access to main registers
	// Set the 3rd bit of the mode_select register to 1.
	uint8_t mode_select = 0x00;
	if (!readRegisterBlocking(hi2c, ICP20100_REG_MODE_SELECT, mode_select)) {
		return false;
	}

	mode_select |= (0x04); // Read previous register and toggle the 3rd bit to preserve previous bits

	if (!writeRegisterWithVerify(hi2c, ICP20100_REG_MODE_SELECT, mode_select)) {
		return false;
	}

	HAL_Delay(4); // blocking delay 4, as required by data sheet

	// Step 6: Unlock main registers by setting the Master_Lock register to 0x1f

	if (!unlockOrLock(hi2c, true)) return false;

	//Step 7: Enable OTP and write switch by setting the config1 register's bits 0 and 1 to 1.
	uint8_t otp_config = 0x00;
	if(HAL_I2C_Mem_Read(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_CONFIG_1, I2C_MEMADD_SIZE_8BIT, &otp_config, 1, HAL_MAX_DELAY) != HAL_OK){
		return false;
	}

	otp_config |= (ICP20100_OTP_ENABLE_BOTH); // Sets bits 011

	if(!writeRegisterWithVerify(hi2c, ICP20100_OTP_CONFIG_1, otp_config)){
		return false;
	}

	HAL_Delay(1); // should be wait 10 microseconds

	//Step 8: Toggle the OTP_DBG2 register bit 8 (reset bit)
	uint8_t reset = 0x00;

	if(HAL_I2C_Mem_Read(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_DBG2, I2C_MEMADD_SIZE_8BIT, &reset, 1, HAL_MAX_DELAY) != HAL_OK){
		return false;
	}

	reset |= (0x80);

	if(!writeRegisterWithVerify(hi2c, ICP20100_OTP_DBG2, reset)){
		return false;
	}

	HAL_Delay(1);

	reset &= ~(0x80);

	if(!writeRegisterWithVerify(hi2c, ICP20100_OTP_DBG2, reset)){
		return false;
	}

	HAL_Delay(1);

	// STEP 9: Program redundant read 
	if (!writeRegisterWithVerify(hi2c, ICP20100_OTP_MRA_LSB, 0x04)) return false;
	if (!writeRegisterWithVerify(hi2c, ICP20100_OTP_MRA_MSB, 0x04)) return false;
	if (!writeRegisterWithVerify(hi2c, ICP20100_OTP_MRB_LSB, 0x21)) return false;
	if (!writeRegisterWithVerify(hi2c, ICP20100_OTP_MRB_MSB, 0x20)) return false;
	if (!writeRegisterWithVerify(hi2c, ICP20100_OTP_MR_LSB, 0x10)) return false;
	if (!writeRegisterWithVerify(hi2c, ICP20100_OTP_MR_MSB, 0x80)) return false;

	// STEP 10: Write address content and read command
	uint8_t command_address = 0xF8;
	// Sets the OTP address to 0xF8 for address to read from
	if(!writeRegisterWithVerify(hi2c, ICP20100_OTP_ADDRESS, command_address)){ return false; }

	command_address = 0x00; //X0010000
	if(HAL_I2C_Mem_Read(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_COMMAND, I2C_MEMADD_SIZE_8BIT, &command_address, 1, HAL_MAX_DELAY) != HAL_OK){ return false; }

	command_address &= ~ICP20100_OTP_COMMAND_FIELD_MASK;
	command_address |= ICP20100_OTP_COMMAND_READ_REQUEST;

	if(!writeRegisterWithVerify(hi2c, ICP20100_OTP_COMMAND, command_address)){ return false; }

	// STEP 11: Wait for OTP read to finish
	if (!waitForOtpStatusClear(hi2c)) {
		return false;
	}

	// STEP 12: Read offset from the OTP_RDATA register
	uint8_t offset = 0x0000;
	if(HAL_I2C_Mem_Read(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_RDATA, I2C_MEMADD_SIZE_8BIT, &offset, 1, HAL_MAX_DELAY) != HAL_OK){ return false; }

	// STEP 13: Write next address

	command_address = 0xF9;
	if(!writeRegisterWithVerify(hi2c, ICP20100_OTP_ADDRESS, command_address)){ return false; }

	command_address = 0x00;
	if(HAL_I2C_Mem_Read(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_COMMAND, I2C_MEMADD_SIZE_8BIT, &command_address, 1, HAL_MAX_DELAY) != HAL_OK){ return false; }

	command_address &= ~ICP20100_OTP_COMMAND_FIELD_MASK;
	command_address |= ICP20100_OTP_COMMAND_READ_REQUEST;

	if(!writeRegisterWithVerify(hi2c, ICP20100_OTP_COMMAND, command_address)){ return false; }

	// STEP 14: Wait for OTP read to finish
	if (!waitForOtpStatusClear(hi2c)) {
		return false;
	}

	// STEP 15: Read gain from OTP_RDATA register
	uint8_t gain = 0x0000;
	if(HAL_I2C_Mem_Read(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_RDATA, I2C_MEMADD_SIZE_8BIT, &gain, 1, HAL_MAX_DELAY) != HAL_OK){ return false; }

	// Step 16: Write next address content

	command_address = 0xFA;
	if(!writeRegisterWithVerify(hi2c, ICP20100_OTP_ADDRESS, command_address)){ return false; }

	command_address = 0x00; //X0010000
	if(HAL_I2C_Mem_Read(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_COMMAND, I2C_MEMADD_SIZE_8BIT, &command_address, 1, HAL_MAX_DELAY) != HAL_OK){ return false; }

	command_address &= ~ICP20100_OTP_COMMAND_FIELD_MASK;
	command_address |= ICP20100_OTP_COMMAND_READ_REQUEST;

	if(!writeRegisterWithVerify(hi2c, ICP20100_OTP_COMMAND, command_address)){ return false; }

	// STEP 17: Wait for OTP read to finish
	if (!waitForOtpStatusClear(hi2c)) {
		return false;
	}

	// STEP 18: Read HFosc
	uint8_t HFosc = 0x00;
	if(HAL_I2C_Mem_Read(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_RDATA, I2C_MEMADD_SIZE_8BIT, &HFosc, 1, HAL_MAX_DELAY) != HAL_OK){ return false; }

	// STEP 19: Disable OTP

	if(HAL_I2C_Mem_Read(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_CONFIG_1, I2C_MEMADD_SIZE_8BIT, &otp_config, 1, HAL_MAX_DELAY) != HAL_OK){ return false; }

	otp_config &= ~(0x03);

	if(!writeRegisterWithVerify(hi2c, ICP20100_OTP_CONFIG_1, otp_config)){ return false; }

	HAL_Delay(1); // should be wait 10 microseconds

	// STEP 20: Write offset to main registers
	uint8_t trim_reg;

	if(HAL_I2C_Mem_Read(hi2c, ICP20100_I2C_ADDR, ICP20100_TRIM1_MSB, I2C_MEMADD_SIZE_8BIT, &trim_reg, 1, HAL_MAX_DELAY) != HAL_OK){ return false; }

	// Clear the 6-bit PEFE_OFFSET_TRIM field (bits 5:0)
	trim_reg &= ~0x3F;

	uint8_t offset_low = offset & 0x3F;
	trim_reg |= offset_low;

	// Write back
	if(!writeRegisterWithVerify(hi2c, ICP20100_TRIM1_MSB, trim_reg)){ return false; }

	// STEP 21: Write gain to main registers
	uint8_t Rdata = 0x00;
	if(HAL_I2C_Mem_Read(hi2c, ICP20100_I2C_ADDR, ICP20100_TRIM2_MSB,
				 I2C_MEMADD_SIZE_8BIT, &Rdata, 1, HAL_MAX_DELAY) != HAL_OK){ return false; }

	Rdata &= ~ICP20100_TRIM2_MSB_GAIN_FIELD_MASK;  // Clear bits 4,5,6
	gain &= ICP20100_GAIN_VALUE_MASK;  // Mask bits 1,2,3, to extract gain value required, as per datasheet
	Rdata |= (gain << ICP20100_TRIM2_MSB_GAIN_SHIFT);  // Set bits 4,5,6 to bits 1,2,3 from gain value

	if(!writeRegisterWithVerify(hi2c, ICP20100_TRIM2_MSB, Rdata)){ return false; }

	// STEP 22: Write HFosc trim value to main registers
	if(!writeRegisterWithVerify(hi2c, ICP20100_TRIM2_LSB, HFosc)){ return false; }

	// STEP 23: Lock main registers
	if (!unlockOrLock(hi2c, true)) {
		return false;
	}

	 // STEP 24: Move to standby
	uint8_t power_mode = 0;

	if(HAL_I2C_Mem_Read(hi2c,
		                  ICP20100_I2C_ADDR,
		                  ICP20100_REG_MODE_SELECT,
		                  I2C_MEMADD_SIZE_8BIT,
		                  &power_mode,
		                  1,
		                  HAL_MAX_DELAY) != HAL_OK){ return false; }

	power_mode &= ~(0x04);

	if(!writeRegisterWithVerify(hi2c, ICP20100_REG_MODE_SELECT, power_mode)){ return false; }

	// STEP 25: Check boot up status to 1, avoid reintialization

	uint8_t boot_config = ICP20100_OTP_STATUS2_BOOTUP;

	if(!writeRegisterWithVerify(hi2c, ICP20100_OTP_STATUS2, boot_config)){ return false; }

	return true;
}

bool Barometer::firWarmupPoll()
{
	uint8_t mode_select = (uint8_t)(0x28); // MEAS_MODE=1, POWER_MODE=0, FIFO_READOUT=0
	uint8_t fifo_fill = 0;
	uint8_t stop_mode = 0x00;
	uint8_t flush_fifo = 0x80;
	uint32_t timeoutMs = 200U;
	
	// Step 1: Configure mode to be in mode 1 and continuous  and start a measuerment

	if (HAL_I2C_Mem_Write(hi2c, ICP20100_I2C_ADDR, ICP20100_REG_MODE_SELECT, I2C_MEMADD_SIZE_8BIT, &mode_select, 1, 10) != HAL_OK) {
		return false;
	}

	// Step 2: Poll for filling up of FIFO fill
	while (timeoutMs-- > 0U) {
		if (HAL_I2C_Mem_Read(hi2c, ICP20100_I2C_ADDR, ICP20100_FIFO_FILL, I2C_MEMADD_SIZE_8BIT, &fifo_fill, 1, 10) != HAL_OK) {
			return false;
		}

		fifo_fill &= 0x1F;
		if (fifo_fill >= 14U) {
			break;
		}

		HAL_Delay(1);
	}

	if (timeoutMs == 0U) {
		return false;
	}

	// Step 3: Stop measuring data 

	if (HAL_I2C_Mem_Write(hi2c, ICP20100_I2C_ADDR, ICP20100_REG_MODE_SELECT, I2C_MEMADD_SIZE_8BIT, &stop_mode, 1, 10) != HAL_OK) {
		return false;
	}

	HAL_Delay(1);

	// Step 4: Flush FIFO filter

	if (HAL_I2C_Mem_Write(hi2c, ICP20100_I2C_ADDR, ICP20100_FIFO_FILL, I2C_MEMADD_SIZE_8BIT, &flush_fifo, 1, 10) != HAL_OK) {
		return false;
	}

	// Step 5: Start measurement 

	if (HAL_I2C_Mem_Write(hi2c, ICP20100_I2C_ADDR, ICP20100_REG_MODE_SELECT, I2C_MEMADD_SIZE_8BIT, &mode_select, 1, 10) != HAL_OK) {
		return false;
	}

	// Step 6: Pass data reading to ReadPressureDMA

	return true;
}

bool Barometer::readRegister(
    uint16_t memAddress,
    uint8_t * pData,
    uint16_t size,
    I2C_HandleTypeDef *hi2c) {
	if (HAL_I2C_Mem_Read_DMA(hi2c, ICP20100_I2C_ADDR, memAddress, I2C_MEMADD_SIZE_8BIT, pData, size) != HAL_OK) {
		return false;
	}

	return true;
}

bool Barometer::writeRegister(
    uint16_t memAddress,
    uint8_t * pData,
    uint16_t size,
    I2C_HandleTypeDef *hi2c) {

    return HAL_I2C_Mem_Write_DMA(hi2c, ICP20100_I2C_ADDR, memAddress, I2C_MEMADD_SIZE_8BIT, pData, size) == HAL_OK;
}

void Barometer::rxCallback() {
	switch(callbackCount) {
		case NotStarted: // Step 1: Start FIFO fill register read via DMA
			dataFilled = 0;
			if (readRegister(ICP20100_FIFO_FILL, &fifoRegister, 1, hi2c)) {
				callbackCount = FifoStarted;
			} else {
				callbackCount = NotStarted;
				initiatedRead = false;
			}
			break;

		case FifoStarted: // Step 2: FIFO read complete. If data ready, read pressure/temp burst.
			fifoRegister &= 0x1F;
			if (fifoRegister > 0) {
				if (readRegister(ICP20100_PRESS_DATA_0, pressTempData, 6, hi2c)) { 
					callbackCount = DataRead;
				} else {
					callbackCount = NotStarted;
					initiatedRead = false;
				}
			} else {
				// Keep polling FIFO until at least one sample is ready.
				callbackCount = NotStarted;
				initiatedRead = false;
			}
			break;

		case DataRead: { // Step 3: Burst read complete. Signal data ready.
			dataFilled = 1;
			callbackCount = NotStarted;
			initiatedRead = false;
			break;
		}

		default:
			callbackCount = NotStarted;
			initiatedRead = false;
			break;
	}
}

bool Barometer::readData(BaroData_t &data)
{
	if (dataFilled) {
		uint32_t press_raw = ((pressTempData[2] & 0x0F) << 16) | (pressTempData[1] << 8) | pressTempData[0];
		uint32_t temp_raw  = ((pressTempData[5] & 0x0F) << 16) | (pressTempData[4] << 8) | pressTempData[3];

		int32_t press_signed = (int32_t)(press_raw & 0xFFFFF);
		if (press_signed & 0x80000) {
			press_signed |= 0xFFF00000;
		}

		int32_t temp_signed = (int32_t)(temp_raw & 0xFFFFF);
		if (temp_signed & 0x80000) {
			temp_signed |= 0xFFF00000;
		}

		data.temperatureData = (float)(((double)temp_signed * 65.0) / 262144.0 + 25.0);
		data.pressureData = (float)(((double)press_signed * 40.0) / 131072.0 + 70.0);
		dataFilled = 0;
		return true;
	}

	if (callbackCount != 0) {
		return false;
	}

	if (HAL_I2C_GetState(hi2c) != HAL_I2C_STATE_READY) {
		return false;
	}

	// Kick off DMA state machine. FIFO polling starts in callback step 1.
	if(!initiatedRead){
		initiatedRead = true;
		rxCallback();
	}

	// Non-blocking: no data ready yet.
	return false;
}

void Barometer::computeAltitude(BaroData_t *data)
{
	if (data != nullptr) {
		/*
		 * Compute altitude from measured temperature and pressure using the
		 * international barometric formula
		 * Steps:
		 *  - Convert temperature from degrees Celsius to Kelvin: T_k = T_C + 273.15
		 *  - Divide by the standard temperature lapse rate (0.0065 K/m) to form
		 *    the scale factor T_k / L (units: meters).
		 *  - Compute the pressure ratio P / P0 where P0 = 101.325 kPa (sea-level standard).
		 *  - Raise the ratio to the exponent ~0.190284 (≈ 1/5.25588), which is derived
		 *    from constants in the barometric equation (R, g, and L).
		 *  - Altitude (m) = (T_k / L) * (1 - (P / P0)^{exponent}).
		 *
		 * Assumptions/notes:
		 *  - `temperatureData` is in °C and `pressureData` is in kPa.
		 *  - This formula gives altitude in meters relative to sea level and is
		 *    an approximation valid under standard-atmosphere conditions.
		 */
		data->altitude = ((data->temperatureData + 273.15f) / 0.0065f) *
						 (1.0f - powf(data->pressureData / 101.325f, 0.190284f));
	}
}

// ============================================================================
// Static helper function implementations
// ============================================================================

static inline bool unlockOrLock(I2C_HandleTypeDef *hi2c, bool doLock) {
    uint8_t value = doLock ? LOCK_VALUE : UNLOCK_VALUE;
    return HAL_I2C_Mem_Write(
        hi2c,
        ICP20100_I2C_ADDR,
        ICP20100_MASTER_LOCK,
        I2C_MEMADD_SIZE_8BIT,
        &value,
        1,
        HAL_MAX_DELAY) == HAL_OK;
}

static inline bool readRegisterBlocking(I2C_HandleTypeDef *hi2c, uint16_t memAddress, uint8_t *pData, uint16_t size, uint32_t timeout) {
    return HAL_I2C_Mem_Read(hi2c, ICP20100_I2C_ADDR, memAddress, I2C_MEMADD_SIZE_8BIT, pData, size, timeout) == HAL_OK;
}

static inline bool readRegisterBlocking(I2C_HandleTypeDef *hi2c, uint16_t memAddress, uint8_t &out, uint32_t timeout) {
    return HAL_I2C_Mem_Read(hi2c, ICP20100_I2C_ADDR, memAddress, I2C_MEMADD_SIZE_8BIT, &out, 1, timeout) == HAL_OK;
}

static inline bool writeRegisterBlocking(I2C_HandleTypeDef *hi2c, uint16_t memAddress, uint8_t value, uint32_t timeout) {
    return HAL_I2C_Mem_Write(hi2c, ICP20100_I2C_ADDR, memAddress, I2C_MEMADD_SIZE_8BIT, &value, 1, timeout) == HAL_OK;
}

static inline bool waitForOtpStatusClear(I2C_HandleTypeDef *hi2c) {
    uint8_t status = 0x01;
    const uint32_t startTick = osKernelGetTickCount();
    const uint32_t timeoutTicks = timeToTicks(OTP_STATUS_POLL_TIMEOUT_MS);
    const uint32_t pollIntervalTicks = timeToTicks(OTP_STATUS_POLL_INTERVAL_MS);

    do {
        if (!readRegisterBlocking(hi2c, ICP20100_OTP_STATUS, status)) {
            return false;
        }

        if ((status & 0x01U) == 0U) {
            return true;
        }

        osDelay(pollIntervalTicks);
    } while ((osKernelGetTickCount() - startTick) < timeoutTicks);

    return false;
}

static inline bool writeRegisterWithVerify(I2C_HandleTypeDef *hi2c, uint16_t memAddress, uint8_t value, uint32_t timeout) {
    if (!writeRegisterBlocking(hi2c, memAddress, value, timeout)) {
        return false;
    }

    uint8_t readBack = 0;
    if (!readRegisterBlocking(hi2c, memAddress, readBack, timeout)) {
        return false;
    }

    return (readBack == value);
}