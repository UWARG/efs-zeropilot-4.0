#include "icp_20100.hpp"
#include "utils.h"

static constexpr uint32_t OTP_STATUS_POLL_TIMEOUT_MS = 1000U;
static constexpr uint32_t OTP_STATUS_POLL_INTERVAL_MS = 1U;

// OTP_STATUS2 boot status bit definitions
static constexpr uint8_t OTP_STATUS2_BOOT_STATUS_BM = 0x01U;     // Bit mask for boot status (bit 0)
static constexpr uint8_t OTP_STATUS2_BOOT_STATUS_VALID = 0x01U;  // Boot is complete
static constexpr uint8_t VERSION_B = 0xB2U;

// OTP_DBG2 register: reset bit (datasheet "bit 8")
static constexpr uint8_t ICP20100_OTP_DBG2_RESET_BM = 0x80U;

// TRIM2_MSB register: Gain field occupies bits 4,5,6
static constexpr uint8_t ICP20100_TRIM2_MSB_GAIN_FIELD_MASK = 0x70U;  // Bits 4,5,6
static constexpr uint8_t ICP20100_TRIM2_MSB_GAIN_SHIFT = 4U;
static constexpr uint8_t ICP20100_GAIN_VALUE_MASK = 0x07U;  // 3-bit gain value from OTP

// OTP_COMMAND register field definitions
static constexpr uint8_t ICP20100_OTP_COMMAND_FIELD_MASK = 0x7FU;  // Bits 0-6
static constexpr uint8_t ICP20100_OTP_COMMAND_READ_REQUEST = 0x10U;  // Bit 4 set to request OTP read

// OTP address-space addresses written to OTP_ADDRESS to select the trim value to read back
static constexpr uint8_t ICP20100_OTP_ADDR_OFFSET = 0xF8U;  // PEFE offset trim
static constexpr uint8_t ICP20100_OTP_ADDR_GAIN = 0xF9U;    // Gain trim
static constexpr uint8_t ICP20100_OTP_ADDR_HFOSC = 0xFAU;   // HFosc trim

// Register Definitions for Mikroe ICP-20100

static constexpr uint16_t ICP20100_I2C_ADDR = (0x63U << 1); // Shift by 1 for HAL
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
static constexpr uint8_t UNLOCK_VALUE = ICP20100_MASTER_UNLOCK_KEY;
static constexpr uint8_t LOCK_VALUE = ICP20100_MASTER_LOCK_KEY;

// MODE_SELECT register field definitions
static constexpr uint8_t ICP20100_MODE_SELECT_POWER_MODE_BM = (1U << 2);      // POWER_MODE bit (active vs standby)
static constexpr uint8_t ICP20100_MODE_SELECT_MEAS_MODE1_CONTINUOUS = 0x28U;  // MEAS_MODE=1, POWER_MODE=0, FIFO_READOUT=0

// TRIM1_MSB register: PEFE_OFFSET_TRIM field occupies bits 5:0
static constexpr uint8_t ICP20100_TRIM1_MSB_OFFSET_FIELD_MASK = 0x3FU;

// OTP_STATUS register: busy bit
static constexpr uint8_t ICP20100_OTP_STATUS_BUSY_BM = 0x01U;

// FIFO_FILL register field definitions
static constexpr uint8_t ICP20100_FIFO_FILL_COUNT_MASK = 0x1FU;  // FIFO sample count occupies bits 4:0
static constexpr uint8_t ICP20100_FIFO_FILL_FLUSH_BM = 0x80U;    // Write 1 to flush the FIFO

// OTP redundant-read programming values (datasheet boot sequence, init step 9)
static constexpr uint8_t ICP20100_OTP_MRA_LSB_VALUE = 0x04U;
static constexpr uint8_t ICP20100_OTP_MRA_MSB_VALUE = 0x04U;
static constexpr uint8_t ICP20100_OTP_MRB_LSB_VALUE = 0x21U;
static constexpr uint8_t ICP20100_OTP_MRB_MSB_VALUE = 0x20U;
static constexpr uint8_t ICP20100_OTP_MR_LSB_VALUE = 0x10U;
static constexpr uint8_t ICP20100_OTP_MR_MSB_VALUE = 0x80U;

// Pressure/temperature burst: 3 pressure + 3 temperature bytes starting at PRESS_DATA_0
static constexpr uint16_t ICP20100_PRESS_TEMP_BURST_SIZE = 6U;

// Raw 20-bit sample decoding
static constexpr uint8_t  ICP20100_RAW_MSB_NIBBLE_MASK = 0x0FU;   // Valid bits in a sample's MSB byte
static constexpr uint32_t ICP20100_RAW_20BIT_MASK = 0xFFFFFU;     // 20-bit sample mask
static constexpr uint32_t ICP20100_RAW_SIGN_BIT = 0x80000U;       // Sign bit of a 20-bit sample (bit 19)
static constexpr uint32_t ICP20100_RAW_SIGN_EXTEND = 0xFFF00000U; // Bits to set when sign-extending to 32 bits

// Datasheet transfer functions (raw -> physical units)
static constexpr double ICP20100_TEMP_SPAN_C = 65.0;
static constexpr double ICP20100_TEMP_DIVISOR = 262144.0;   // 2^18
static constexpr double ICP20100_TEMP_OFFSET_C = 25.0;
static constexpr double ICP20100_PRESS_SPAN_KPA = 40.0;
static constexpr double ICP20100_PRESS_DIVISOR = 131072.0;  // 2^17
static constexpr double ICP20100_PRESS_OFFSET_KPA = 70.0;

// Barometric altitude formula constants
static constexpr float ICP20100_KELVIN_OFFSET = 273.15f;
static constexpr float ICP20100_TEMP_LAPSE_RATE = 0.0065f;          // K/m
static constexpr float ICP20100_SEA_LEVEL_PRESSURE_KPA = 101.325f;
static constexpr float ICP20100_BAROMETRIC_EXPONENT = 0.190284f;

// Timing
static constexpr uint32_t ICP20100_POWER_MODE_DELAY_MS = 4U;        // Settle after entering power mode (datasheet)
static constexpr uint32_t ICP20100_SHORT_DELAY_MS = 1U;             // Brief settle / poll delay
static constexpr uint32_t ICP20100_FIR_WARMUP_TIMEOUT_MS = 200U;    // FIR warm-up FIFO-fill timeout
static constexpr uint8_t  ICP20100_FIR_WARMUP_FIFO_THRESHOLD = 14U; // Samples to accumulate during warm-up
static constexpr uint32_t ICP20100_FIR_WARMUP_I2C_TIMEOUT_MS = 10U; // Blocking I2C timeout during warm-up

// Forward declarations of static helper functions
static bool unlockOrLock(I2C_HandleTypeDef *hi2c, bool doLock);
static bool readRegisterBlocking(I2C_HandleTypeDef *hi2c, uint16_t memAddress, uint8_t *pData, uint16_t size, uint32_t timeout = HAL_MAX_DELAY);
static bool readRegisterBlocking(I2C_HandleTypeDef *hi2c, uint16_t memAddress, uint8_t &out, uint32_t timeout = HAL_MAX_DELAY);
static bool writeRegisterBlocking(I2C_HandleTypeDef *hi2c, uint16_t memAddress, uint8_t value, uint32_t timeout = HAL_MAX_DELAY);
static bool waitForOtpStatusClear(I2C_HandleTypeDef *hi2c);
static bool writeRegisterWithVerify(I2C_HandleTypeDef *hi2c, uint16_t memAddress, uint8_t value, uint32_t timeout = HAL_MAX_DELAY);

Barometer::Barometer(I2C_HandleTypeDef *hi2c) :
	hi2c(hi2c), callbackState(NotStarted), fifoRegister(0) {}
	
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
        return firWarmupPoll();
    }

	// Step 4: Check boot up status from OTP_Status2 register. Check specifically bit 0.
	uint8_t bootStatus = 0x00;
	if (!readRegisterBlocking(hi2c, ICP20100_OTP_STATUS2, bootStatus)) {
		return false;
	}

	// Mask boot status register to only read the 0th bit
	bootStatus &= OTP_STATUS2_BOOT_STATUS_BM;

	if (bootStatus == OTP_STATUS2_BOOT_STATUS_VALID) { // Initialization done, barometer did not go through power cycle.
		return firWarmupPoll();
	}

	// Step 5: Bring ASIC into power mode to get access to main registers
	// Set the 3rd bit of the modeSelect register to 1.
	uint8_t modeSelect = 0x00;
	if (!readRegisterBlocking(hi2c, ICP20100_REG_MODE_SELECT, modeSelect)) {
		return false;
	}

	modeSelect |= ICP20100_MODE_SELECT_POWER_MODE_BM; // Read previous register and toggle the power-mode bit to preserve previous bits

	if (!writeRegisterWithVerify(hi2c, ICP20100_REG_MODE_SELECT, modeSelect)) {
		return false;
	}

	HAL_Delay(ICP20100_POWER_MODE_DELAY_MS); // blocking delay, as required by data sheet

	// Step 6: Unlock main registers by setting the Master_Lock register to 0x1f

	if (!unlockOrLock(hi2c, true)) return false;

	//Step 7: Enable OTP and write switch by setting the config1 register's bits 0 and 1 to 1.
	uint8_t otpConfig = 0x00;
	if (HAL_I2C_Mem_Read(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_CONFIG_1, I2C_MEMADD_SIZE_8BIT, &otpConfig, 1, HAL_MAX_DELAY) != HAL_OK) {
		return false;
	}

	otpConfig |= (ICP20100_OTP_ENABLE_BOTH); // Sets bits 011

	if (!writeRegisterWithVerify(hi2c, ICP20100_OTP_CONFIG_1, otpConfig)) {
		return false;
	}

	HAL_Delay(ICP20100_SHORT_DELAY_MS); // should be wait 10 microseconds

	//Step 8: Toggle the OTP_DBG2 register bit 8 (reset bit)
	uint8_t reset = 0x00;

	if (HAL_I2C_Mem_Read(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_DBG2, I2C_MEMADD_SIZE_8BIT, &reset, 1, HAL_MAX_DELAY) != HAL_OK) {
		return false;
	}

	reset |= (ICP20100_OTP_DBG2_RESET_BM);

	if (!writeRegisterWithVerify(hi2c, ICP20100_OTP_DBG2, reset)) {
		return false;
	}

	HAL_Delay(ICP20100_SHORT_DELAY_MS);

	reset &= ~(ICP20100_OTP_DBG2_RESET_BM);

	if (!writeRegisterWithVerify(hi2c, ICP20100_OTP_DBG2, reset)) {
		return false;
	}

	HAL_Delay(ICP20100_SHORT_DELAY_MS);

	// STEP 9: Program redundant read 
	if (!writeRegisterWithVerify(hi2c, ICP20100_OTP_MRA_LSB, ICP20100_OTP_MRA_LSB_VALUE)) return false;
	if (!writeRegisterWithVerify(hi2c, ICP20100_OTP_MRA_MSB, ICP20100_OTP_MRA_MSB_VALUE)) return false;
	if (!writeRegisterWithVerify(hi2c, ICP20100_OTP_MRB_LSB, ICP20100_OTP_MRB_LSB_VALUE)) return false;
	if (!writeRegisterWithVerify(hi2c, ICP20100_OTP_MRB_MSB, ICP20100_OTP_MRB_MSB_VALUE)) return false;
	if (!writeRegisterWithVerify(hi2c, ICP20100_OTP_MR_LSB, ICP20100_OTP_MR_LSB_VALUE)) return false;
	if (!writeRegisterWithVerify(hi2c, ICP20100_OTP_MR_MSB, ICP20100_OTP_MR_MSB_VALUE)) return false;

	// STEP 10: Write address content and read command
	uint8_t commandAddress = ICP20100_OTP_ADDR_OFFSET;
	// Select the OTP offset-trim address to read from
	if (!writeRegisterWithVerify(hi2c, ICP20100_OTP_ADDRESS, commandAddress)) return false; 

	commandAddress = 0x00; //X0010000
	if (HAL_I2C_Mem_Read(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_COMMAND, I2C_MEMADD_SIZE_8BIT, &commandAddress, 1, HAL_MAX_DELAY) != HAL_OK) return false; 

	commandAddress &= ~ICP20100_OTP_COMMAND_FIELD_MASK;
	commandAddress |= ICP20100_OTP_COMMAND_READ_REQUEST;

	if (!writeRegisterWithVerify(hi2c, ICP20100_OTP_COMMAND, commandAddress)) return false; 

	// STEP 11: Wait for OTP read to finish
	if (!waitForOtpStatusClear(hi2c)) return false;

	// STEP 12: Read offset from the OTP_RDATA register
	uint8_t offset = 0;
	if (HAL_I2C_Mem_Read(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_RDATA, I2C_MEMADD_SIZE_8BIT, &offset, 1, HAL_MAX_DELAY) != HAL_OK) return false; 

	// STEP 13: Write next address
	commandAddress = ICP20100_OTP_ADDR_GAIN;
	if (!writeRegisterWithVerify(hi2c, ICP20100_OTP_ADDRESS, commandAddress)) return false; 

	commandAddress = 0x00;
	if (HAL_I2C_Mem_Read(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_COMMAND, I2C_MEMADD_SIZE_8BIT, &commandAddress, 1, HAL_MAX_DELAY) != HAL_OK) return false; 

	commandAddress &= ~ICP20100_OTP_COMMAND_FIELD_MASK;
	commandAddress |= ICP20100_OTP_COMMAND_READ_REQUEST;

	if (!writeRegisterWithVerify(hi2c, ICP20100_OTP_COMMAND, commandAddress)) return false; 

	// STEP 14: Wait for OTP read to finish
	if (!waitForOtpStatusClear(hi2c)) return false;

	// STEP 15: Read gain from OTP_RDATA register
	uint8_t gain = 0x0000;
	if (HAL_I2C_Mem_Read(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_RDATA, I2C_MEMADD_SIZE_8BIT, &gain, 1, HAL_MAX_DELAY) != HAL_OK) return false; 

	// Step 16: Write next address content

	commandAddress = ICP20100_OTP_ADDR_HFOSC;
	if (!writeRegisterWithVerify(hi2c, ICP20100_OTP_ADDRESS, commandAddress)) return false; 
	commandAddress = 0x00; //X0010000
	if (HAL_I2C_Mem_Read(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_COMMAND, I2C_MEMADD_SIZE_8BIT, &commandAddress, 1, HAL_MAX_DELAY) != HAL_OK){ return false; }

	commandAddress &= ~ICP20100_OTP_COMMAND_FIELD_MASK;
	commandAddress |= ICP20100_OTP_COMMAND_READ_REQUEST;

	if (!writeRegisterWithVerify(hi2c, ICP20100_OTP_COMMAND, commandAddress)) return false; 

	// STEP 17: Wait for OTP read to finish
	if (!waitForOtpStatusClear(hi2c)) {
		return false;
	}

	// STEP 18: Read HFosc
	uint8_t HFosc = 0x00;
	if (HAL_I2C_Mem_Read(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_RDATA, I2C_MEMADD_SIZE_8BIT, &HFosc, 1, HAL_MAX_DELAY) != HAL_OK) return false; 

	// STEP 19: Disable OTP

	if (HAL_I2C_Mem_Read(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_CONFIG_1, I2C_MEMADD_SIZE_8BIT, &otpConfig, 1, HAL_MAX_DELAY) != HAL_OK) return false; 
	otpConfig &= ~(ICP20100_OTP_ENABLE_BOTH);

	if (!writeRegisterWithVerify(hi2c, ICP20100_OTP_CONFIG_1, otpConfig)) return false; 

	HAL_Delay(ICP20100_SHORT_DELAY_MS); // Needs to wait atleast 10 microseconds, waits 1 milisecond

	// STEP 20: Write offset to main registers
	uint8_t trimReg;

	if (HAL_I2C_Mem_Read(hi2c, ICP20100_I2C_ADDR, ICP20100_TRIM1_MSB, I2C_MEMADD_SIZE_8BIT, &trimReg, 1, HAL_MAX_DELAY) != HAL_OK) return false; 

	// Clear the 6-bit PEFE_OFFSET_TRIM field (bits 5:0)
	trimReg &= ~ICP20100_TRIM1_MSB_OFFSET_FIELD_MASK;

	uint8_t offsetLow = offset & ICP20100_TRIM1_MSB_OFFSET_FIELD_MASK;
	trimReg |= offsetLow;

	// Write back
	if (!writeRegisterWithVerify(hi2c, ICP20100_TRIM1_MSB, trimReg)) return false; 

	// STEP 21: Write gain to main registers
	uint8_t rData = 0x00;
	if (HAL_I2C_Mem_Read(hi2c, ICP20100_I2C_ADDR, ICP20100_TRIM2_MSB,
				 I2C_MEMADD_SIZE_8BIT, &Rdata, 1, HAL_MAX_DELAY) != HAL_OK ) return false; 

	rData &= ~ICP20100_TRIM2_MSB_GAIN_FIELD_MASK;  // Clear bits 4, 5, 6
	gain &= ICP20100_GAIN_VALUE_MASK;  // Mask bits 1, 2, 3 to extract gain value required, as per datasheet
	rData |= (gain << ICP20100_TRIM2_MSB_GAIN_SHIFT);  // Set bits 4, 5, 6 to bits 1, 2, 3 from gain value

	if (!writeRegisterWithVerify(hi2c, ICP20100_TRIM2_MSB, rData)) return false; 

	// STEP 22: Write HFosc trim value to main registers
	if (!writeRegisterWithVerify(hi2c, ICP20100_TRIM2_LSB, HFosc)) return false; 

	// STEP 23: Lock main registers
	if (!unlockOrLock(hi2c, true)) {
		return false;
	}

	 // STEP 24: Move to standby
	uint8_t powerMode = 0;

	if (HAL_I2C_Mem_Read(hi2c,
		                 ICP20100_I2C_ADDR,
		                 ICP20100_REG_MODE_SELECT,
		                 I2C_MEMADD_SIZE_8BIT,
		                 &powerMode,
		                 1,
		                 HAL_MAX_DELAY) != HAL_OK) return false; 

	powerMode &= ~(ICP20100_MODE_SELECT_POWER_MODE_BM);

	if (!writeRegisterWithVerify(hi2c, ICP20100_REG_MODE_SELECT, powerMode)) return false; 

	// STEP 25: Check boot up status to 1, avoid reintialization

	uint8_t bootConfig = ICP20100_OTP_STATUS2_BOOTUP;

	if (!writeRegisterWithVerify(hi2c, ICP20100_OTP_STATUS2, bootConfig)) return false; 

	return firWarmupPoll();
}

bool Barometer::firWarmupPoll() {
	uint8_t modeSelect = ICP20100_MODE_SELECT_MEAS_MODE1_CONTINUOUS;
	uint8_t fifoFill = 0;
	uint8_t stopMode = 0x00;
	uint8_t flushFifo = ICP20100_FIFO_FILL_FLUSH_BM;
	
	// Step 1: Configure mode to be in mode 1 and continuous  and start a measuerment
	if (HAL_I2C_Mem_Write(hi2c, ICP20100_I2C_ADDR, ICP20100_REG_MODE_SELECT, I2C_MEMADD_SIZE_8BIT, &modeSelect, 1, ICP20100_FIR_WARMUP_I2C_TIMEOUT_MS) != HAL_OK) {
		return false;
	}

	// Step 2: Poll for the FIFO to fill
	const uint32_t startMs = HAL_GetTick();
	bool fifoReady = false;

	while ((HAL_GetTick() - startMs) < ICP20100_FIR_WARMUP_TIMEOUT_MS) {
		if (HAL_I2C_Mem_Read(hi2c, 
							 ICP20100_I2C_ADDR, 
							 ICP20100_FIFO_FILL, 
							 I2C_MEMADD_SIZE_8BIT, 
							 &fifoFill, 
							 1, 
							 ICP20100_FIR_WARMUP_I2C_TIMEOUT_MS) != HAL_OK) return false;
		

		fifoFill &= ICP20100_FIFO_FILL_COUNT_MASK;
		if (fifoFill >= ICP20100_FIR_WARMUP_FIFO_THRESHOLD) {
			fifoReady = true;
			break;
		}

		HAL_Delay(ICP20100_SHORT_DELAY_MS);
	}

	if (!fifoReady) {
		return false;
	}

	// Step 3: Stop measuring data 

	if (HAL_I2C_Mem_Write(hi2c, ICP20100_I2C_ADDR, ICP20100_REG_MODE_SELECT, I2C_MEMADD_SIZE_8BIT, &stopMode, 1, ICP20100_FIR_WARMUP_I2C_TIMEOUT_MS) != HAL_OK) {
		return false;
	}

	HAL_Delay(ICP20100_SHORT_DELAY_MS);

	// Step 4: Flush FIFO filter

	if (HAL_I2C_Mem_Write(hi2c, ICP20100_I2C_ADDR, ICP20100_FIFO_FILL, I2C_MEMADD_SIZE_8BIT, &flushFifo, 1, ICP20100_FIR_WARMUP_I2C_TIMEOUT_MS) != HAL_OK) {
		return false;
	}

	// Step 5: Start measurement 

	if (HAL_I2C_Mem_Write(hi2c, ICP20100_I2C_ADDR, ICP20100_REG_MODE_SELECT, I2C_MEMADD_SIZE_8BIT, &modeSelect, 1, ICP20100_FIR_WARMUP_I2C_TIMEOUT_MS) != HAL_OK) {
		return false;
	}

	// Step 6: Pass data reading to ReadPressureDMA

	return true;
}

bool Barometer::readRegister(
    uint16_t memAddress,
    uint8_t * pData,
    uint16_t size) {
	if (HAL_I2C_Mem_Read_DMA(hi2c, ICP20100_I2C_ADDR, memAddress, I2C_MEMADD_SIZE_8BIT, pData, size) != HAL_OK) {
		return false;
	}

	return true;
}

bool Barometer::writeRegister(
    uint16_t memAddress,
    uint8_t * pData,
    uint16_t size) {

    return HAL_I2C_Mem_Write_DMA(hi2c, ICP20100_I2C_ADDR, memAddress, I2C_MEMADD_SIZE_8BIT, pData, size) == HAL_OK;
}

void Barometer::rxCallback() {
	switch (callbackState) {
		case NOT_STARTED: { // Step 1: Start FIFO fill register read via DMA
			dataFilled = 0;
			if (readRegister(ICP20100_FIFO_FILL, &fifoRegister, 1)) {
				callbackState = FIFO_STARTED;
			} else {
				callbackState = NOT_STARTED;
				initiatedRead = false;
			}
			break;
		}

		case FIFO_STARTED: { // Step 2: FIFO read complete. If data ready, read pressure/temp burst.
			fifoRegister &= ICP20100_FIFO_FILL_COUNT_MASK;
			if (fifoRegister > 0) {
				if (readRegister(ICP20100_PRESS_DATA_0, pressTempData, ICP20100_PRESS_TEMP_BURST_SIZE)) {
					callbackState = DATA_READ;
				} else {
					callbackState = NOT_STARTED;
					initiatedRead = false;
				}
			} else {
				// Keep polling FIFO until at least one sample is ready.
				callbackState = NOT_STARTED;
				initiatedRead = false;
			}
			break;
		}

		case DATA_READ: { // Step 3: Burst read complete. Signal data ready.
			dataFilled = 1;
			callbackState = NOT_STARTED;
			initiatedRead = false;
			break;
		}

		default: {
			callbackState = NOT_STARTED;
			initiatedRead = false;
			break;
		}
	}
}

bool Barometer::readData(BaroData_t &data)
{
	if (dataFilled) {
		uint32_t pressRaw = ((pressTempData[2] & ICP20100_RAW_MSB_NIBBLE_MASK) << 16) | (pressTempData[1] << 8) | pressTempData[0];
		uint32_t tempRaw  = ((pressTempData[5] & ICP20100_RAW_MSB_NIBBLE_MASK) << 16) | (pressTempData[4] << 8) | pressTempData[3];

		int32_t pressSigned = (int32_t)(pressRaw & ICP20100_RAW_20BIT_MASK);
		if (pressSigned & ICP20100_RAW_SIGN_BIT) {
			pressSigned |= ICP20100_RAW_SIGN_EXTEND;
		}

		int32_t tempSigned = (int32_t)(tempRaw & ICP20100_RAW_20BIT_MASK);
		if (tempSigned & ICP20100_RAW_SIGN_BIT) {
			tempSigned |= ICP20100_RAW_SIGN_EXTEND;
		}

		data.temperatureC = (float)(((double)tempSigned * ICP20100_TEMP_SPAN_C) / ICP20100_TEMP_DIVISOR + ICP20100_TEMP_OFFSET_C);
		data.pressureKPa = (float)(((double)pressSigned * ICP20100_PRESS_SPAN_KPA) / ICP20100_PRESS_DIVISOR + ICP20100_PRESS_OFFSET_KPA);
		data.altitude = ((data.temperatureC + ICP20100_KELVIN_OFFSET) / ICP20100_TEMP_LAPSE_RATE) *
						 (1.0f - powf(data.pressureKPa / ICP20100_SEA_LEVEL_PRESSURE_KPA, ICP20100_BAROMETRIC_EXPONENT));
		dataFilled = 0;
		initiatedRead = true;
		rxCallback();
		return true;
	}

	if (callbackState != 0) {
		return false;
	}

	if (HAL_I2C_GetState(hi2c) != HAL_I2C_STATE_READY) {
		return false;
	}

	// Kick off DMA state machine. FIFO polling starts in callback step 1.
	if (!initiatedRead){
		initiatedRead = true;
		rxCallback();
	}

	// Non-blocking: no data ready yet.
	return false;
}

I2C_HandleTypeDef* Barometer::getI2C() {
	return hi2c;
}

// ============================================================================
// Static helper function implementations
// ============================================================================

static inline bool unlockOrLock(I2C_HandleTypeDef *hi2c, bool doLock) {
    uint8_t value = doLock ? LOCK_VALUE : UNLOCK_VALUE;
    HAL_StatusTypeDef status =  HAL_I2C_Mem_Write(
        hi2c,
        ICP20100_I2C_ADDR,
        ICP20100_MASTER_LOCK,
        I2C_MEMADD_SIZE_8BIT,
        &value,
        1,
        HAL_MAX_DELAY);
	return status == HAL_OK;
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
    uint8_t status = ICP20100_OTP_STATUS_BUSY_BM;
    const uint32_t startTick = osKernelGetTickCount();
    const uint32_t timeoutTicks = timeToTicks(OTP_STATUS_POLL_TIMEOUT_MS);
    const uint32_t pollIntervalTicks = timeToTicks(OTP_STATUS_POLL_INTERVAL_MS);

    do {
        if (!readRegisterBlocking(hi2c, ICP20100_OTP_STATUS, status)) {
            return false;
        }

        if ((status & ICP20100_OTP_STATUS_BUSY_BM) == 0U) {
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
