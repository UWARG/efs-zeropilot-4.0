#include "icp_20100.hpp"
#include "utils.h"

static constexpr uint32_t OTP_STATUS_POLL_TIMEOUT_MS = 1000U;
static constexpr uint32_t OTP_STATUS_POLL_INTERVAL_MS = 1U;

// OTP_STATUS2 boot status bit definitions
static constexpr uint8_t OTP_STATUS2_BOOT_STATUS_BM = 0x01U;     // Bit mask for boot status (bit 0)
static constexpr uint8_t OTP_STATUS2_BOOT_STATUS_VALID = 0x01U;  // Boot is complete
static constexpr uint8_t VERSION_A = 0x00U;
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
static constexpr uint8_t ICP20100_OTP_ADDR_HFOSC = 0xFAU;   // HfOsc trim

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
static constexpr uint8_t UNLOCK_VALUE = ICP20100_MASTER_UNLOCK_KEY;
static constexpr uint8_t LOCK_VALUE = ICP20100_MASTER_LOCK_KEY;

// MODE_SELECT register field definitions
static constexpr uint8_t ICP20100_MODE_SELECT_POWER_MODE_BM = (1U << 2);      // POWER_MODE bit (active vs standby)
static constexpr uint8_t ICP20100_MODE_SELECT_MEAS_MODE1_CONTINUOUS = 0x28U;  // MEAS_CONFIG=001(mode 1), FORCED_MEAS_TRIGGER=0, MEAS_MODE=1, POWER_MODE=0, FIFO_READOUT=0
static constexpr uint8_t ICP20100_MODE_SELECT_MEAS_MODE0_CONTINUOUS = 0x08U;  // MEAS_CONFIG=0(mode 0), FORCED_MEAS_TRIGGER=0, MEAS_MODE=1, POWER_MODE=0, FIFO_READOUT=0

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
static constexpr float ICP20100_STD_SEA_LEVEL_TEMP_K = 288.15f;

// Timing
static constexpr uint32_t ICP20100_POWER_MODE_DELAY_MS = 4U;        // Settle after entering power mode (datasheet)
static constexpr uint32_t ICP20100_SHORT_DELAY_MS = 1U;             // Brief settle / poll delay
static constexpr uint32_t ICP20100_FIR_WARMUP_TIMEOUT_MS = 1000U;   // FIR warm-up FIFO-fill timeout
static constexpr uint8_t  ICP20100_FIR_WARMUP_FIFO_THRESHOLD = 14U; // Samples to accumulate during warm-up
static constexpr uint32_t ICP20100_FIR_WARMUP_I2C_TIMEOUT_MS = 10U; // Blocking I2C timeout during warm-up
static constexpr uint32_t ICP20100_POWER_ON_TIMEOUT_MS = 100U;      // Max wait for the ASIC to power up and report its version
static constexpr uint32_t ICP20100_POWER_ON_POLL_INTERVAL_MS = 5U;  // Poll interval while waiting for device to be ready
static constexpr uint32_t ICP20100_MODE_SYNC_TIMEOUT_MS = 100U;     // Max wait for MODE_SELECT

// Forward declarations of static helper functions
static ZP_Error halToZpError(HAL_StatusTypeDef status);
static ZP_Error unlockOrLock(I2C_HandleTypeDef *hi2c, bool doLock);
static ZP_Error readRegisterBlocking(I2C_HandleTypeDef *hi2c, uint16_t memAddress, uint8_t &out, uint32_t timeout = HAL_MAX_DELAY);
static ZP_Error writeRegisterBlocking(I2C_HandleTypeDef *hi2c, uint16_t memAddress, uint8_t value, uint32_t timeout = HAL_MAX_DELAY);
static ZP_Error waitForOtpStatusClear(I2C_HandleTypeDef *hi2c);
static ZP_Error writeRegisterWithVerify(I2C_HandleTypeDef *hi2c, uint16_t memAddress, uint8_t value, uint32_t timeout = HAL_MAX_DELAY);
static ZP_Error waitForModeSync(I2C_HandleTypeDef *hi2c);
static ZP_Error waitForDeviceReady(I2C_HandleTypeDef *hi2c, uint8_t &version);
static ZP_Error readOtpByte(I2C_HandleTypeDef *hi2c, uint8_t otpAddress, uint8_t &out);

Barometer::Barometer(I2C_HandleTypeDef *hi2c) :
	hi2c(hi2c), callbackState(NOT_STARTED), fifoRegister(0) {}

ZP_Error Barometer::init() {
	ZP_Error status = ZP_ERROR_OK;

	// Steps 1-3: Wait for the ASIC to finish its power-on boot, dummy I2C writes, and read the version register
	uint8_t version = 0x00;
	status = waitForDeviceReady(hi2c, version);
	if (status != ZP_ERROR_OK) return status;

	if (version == VERSION_B) {
		return firWarmupPoll();
	}

	// Step 4: Check boot up status from OTP_Status2 register. Check specifically bit 0.
	uint8_t bootStatus = 0x00;
	status = readRegisterBlocking(hi2c, ICP20100_OTP_STATUS2, bootStatus);
	if (status != ZP_ERROR_OK) return status;

	// Mask boot status register to only read the 0th bit
	bootStatus &= OTP_STATUS2_BOOT_STATUS_BM;

	if (bootStatus == OTP_STATUS2_BOOT_STATUS_VALID) { // Initialization done, barometer did not go through power cycle.
		return firWarmupPoll();
	}

	// Step 5: Bring ASIC into power mode to get access to main registers
	// Set the 3rd bit of the modeSelect register to 1.
	uint8_t modeSelect = 0x00;
	status = readRegisterBlocking(hi2c, ICP20100_REG_MODE_SELECT, modeSelect);
	if (status != ZP_ERROR_OK) return status;

	modeSelect |= ICP20100_MODE_SELECT_POWER_MODE_BM; // Read previous register and toggle the power-mode bit to preserve previous bits

	status = waitForModeSync(hi2c); // MODE_SELECT is only writable once DEVICE_STATUS's MODE_SYNC_STATUS is set
	if (status != ZP_ERROR_OK) return status;

	status = writeRegisterWithVerify(hi2c, ICP20100_REG_MODE_SELECT, modeSelect);
	if (status != ZP_ERROR_OK) return status;

	HAL_Delay(ICP20100_POWER_MODE_DELAY_MS); // blocking delay, as required by data sheet

	// Step 6: Unlock main registers by setting the Master_Lock register to 0x1f
	status = unlockOrLock(hi2c, false);
	if (status != ZP_ERROR_OK) return status;

	// Step 7: Enable OTP and write switch by setting the config1 register's bits 0 and 1 to 1.
	uint8_t otpConfig = 0x00;
	status = readRegisterBlocking(hi2c, ICP20100_OTP_CONFIG_1, otpConfig);
	if (status != ZP_ERROR_OK) return status;

	otpConfig |= (ICP20100_OTP_ENABLE_BOTH); // Sets bits 011

	status = writeRegisterWithVerify(hi2c, ICP20100_OTP_CONFIG_1, otpConfig);
	if (status != ZP_ERROR_OK) return status;

	HAL_Delay(ICP20100_SHORT_DELAY_MS); // should be wait 10 microseconds

	// Step 8: Toggle the OTP_DBG2 register bit 8 (reset bit)
	uint8_t reset = 0x00;
	status = readRegisterBlocking(hi2c, ICP20100_OTP_DBG2, reset);
	if (status != ZP_ERROR_OK) return status;

	reset |= (ICP20100_OTP_DBG2_RESET_BM);

	status = writeRegisterWithVerify(hi2c, ICP20100_OTP_DBG2, reset);
	if (status != ZP_ERROR_OK) return status;

	HAL_Delay(ICP20100_SHORT_DELAY_MS);

	reset &= ~(ICP20100_OTP_DBG2_RESET_BM);

	status = writeRegisterWithVerify(hi2c, ICP20100_OTP_DBG2, reset);
	if (status != ZP_ERROR_OK) return status;

	HAL_Delay(ICP20100_SHORT_DELAY_MS);

	// Step 9: Program redundant read
	status |= writeRegisterWithVerify(hi2c, ICP20100_OTP_MRA_LSB, ICP20100_OTP_MRA_LSB_VALUE);
	status |= writeRegisterWithVerify(hi2c, ICP20100_OTP_MRA_MSB, ICP20100_OTP_MRA_MSB_VALUE);
	status |= writeRegisterWithVerify(hi2c, ICP20100_OTP_MRB_LSB, ICP20100_OTP_MRB_LSB_VALUE);
	status |= writeRegisterWithVerify(hi2c, ICP20100_OTP_MRB_MSB, ICP20100_OTP_MRB_MSB_VALUE);
	status |= writeRegisterWithVerify(hi2c, ICP20100_OTP_MR_LSB, ICP20100_OTP_MR_LSB_VALUE);
	status |= writeRegisterWithVerify(hi2c, ICP20100_OTP_MR_MSB, ICP20100_OTP_MR_MSB_VALUE);
	if (status != ZP_ERROR_OK) return status;

	// Steps 10-18: Read the offset, gain and HfOsc trims out of OTP
	uint8_t offset = 0x00;
	status = readOtpByte(hi2c, ICP20100_OTP_ADDR_OFFSET, offset);
	if (status != ZP_ERROR_OK) return status;

	uint8_t gain = 0x00;
	status = readOtpByte(hi2c, ICP20100_OTP_ADDR_GAIN, gain);
	if (status != ZP_ERROR_OK) return status;

	uint8_t HfOsc = 0x00;
	status = readOtpByte(hi2c, ICP20100_OTP_ADDR_HFOSC, HfOsc);
	if (status != ZP_ERROR_OK) return status;

	// Step 19: Disable OTP
	status = readRegisterBlocking(hi2c, ICP20100_OTP_CONFIG_1, otpConfig);
	if (status != ZP_ERROR_OK) return status;

	otpConfig &= ~(ICP20100_OTP_ENABLE_BOTH);

	status = writeRegisterWithVerify(hi2c, ICP20100_OTP_CONFIG_1, otpConfig);
	if (status != ZP_ERROR_OK) return status;

	HAL_Delay(ICP20100_SHORT_DELAY_MS); // Needs to wait atleast 10 microseconds, waits 1 milisecond

	// Step 20: Write offset to main registers
	uint8_t trimReg = 0x00;
	status = readRegisterBlocking(hi2c, ICP20100_TRIM1_MSB, trimReg);
	if (status != ZP_ERROR_OK) return status;

	// Clear the 6-bit PEFE_OFFSET_TRIM field (bits 5:0)
	trimReg &= ~ICP20100_TRIM1_MSB_OFFSET_FIELD_MASK;

	uint8_t offsetLow = offset & ICP20100_TRIM1_MSB_OFFSET_FIELD_MASK;
	trimReg |= offsetLow;

	status = writeRegisterWithVerify(hi2c, ICP20100_TRIM1_MSB, trimReg);
	if (status != ZP_ERROR_OK) return status;

	// Step 21: Write gain to main registers
	uint8_t rData = 0x00;
	status = readRegisterBlocking(hi2c, ICP20100_TRIM2_MSB, rData);
	if (status != ZP_ERROR_OK) return status;

	rData &= ~ICP20100_TRIM2_MSB_GAIN_FIELD_MASK;  // Clear bits 4, 5, 6
	gain &= ICP20100_GAIN_VALUE_MASK;  // Mask bits 1, 2, 3 to extract gain value required, as per datasheet
	rData |= (gain << ICP20100_TRIM2_MSB_GAIN_SHIFT);  // Set bits 4, 5, 6 to bits 1, 2, 3 from gain value

	status = writeRegisterWithVerify(hi2c, ICP20100_TRIM2_MSB, rData);
	if (status != ZP_ERROR_OK) return status;

	// Step 22: Write HfOsc trim value to main registers
	status = writeRegisterWithVerify(hi2c, ICP20100_TRIM2_LSB, HfOsc);
	if (status != ZP_ERROR_OK) return status;

	// Step 23: Lock main registers
	status = unlockOrLock(hi2c, true);
	if (status != ZP_ERROR_OK) return status;

	// Step 24: Move to standby
	uint8_t powerMode = 0;
	status = readRegisterBlocking(hi2c, ICP20100_REG_MODE_SELECT, powerMode);
	if (status != ZP_ERROR_OK) return status;

	powerMode &= ~(ICP20100_MODE_SELECT_POWER_MODE_BM);

	status = waitForModeSync(hi2c);
	if (status != ZP_ERROR_OK) return status;

	status = writeRegisterWithVerify(hi2c, ICP20100_REG_MODE_SELECT, powerMode);
	if (status != ZP_ERROR_OK) return status;

	// Step 25: Check boot up status to 1, avoid reintialization
	status = writeRegisterWithVerify(hi2c, ICP20100_OTP_STATUS2, ICP20100_OTP_STATUS2_BOOTUP);
	if (status != ZP_ERROR_OK) return status;

	return firWarmupPoll();
}

ZP_Error Barometer::firWarmupPoll() {
	ZP_Error status = ZP_ERROR_OK;

	// Step 1: Configure mode to be in mode 1 and continuous and start a measuerment
	status = waitForModeSync(hi2c);
	if (status != ZP_ERROR_OK) return status;

	status = writeRegisterBlocking(hi2c, ICP20100_REG_MODE_SELECT, ICP20100_MODE_SELECT_MEAS_MODE0_CONTINUOUS, ICP20100_FIR_WARMUP_I2C_TIMEOUT_MS);
	if (status != ZP_ERROR_OK) return status;

	// Step 2: Poll for the FIFO to fill
	const uint32_t startMs = HAL_GetTick();
	bool fifoReady = false;

	while ((HAL_GetTick() - startMs) < ICP20100_FIR_WARMUP_TIMEOUT_MS) {
		uint8_t fifoFill = 0;
		status = readRegisterBlocking(hi2c, ICP20100_FIFO_FILL, fifoFill, ICP20100_FIR_WARMUP_I2C_TIMEOUT_MS);
		if (status != ZP_ERROR_OK) return status;

		fifoFill &= ICP20100_FIFO_FILL_COUNT_MASK;
		if (fifoFill >= ICP20100_FIR_WARMUP_FIFO_THRESHOLD) {
			fifoReady = true;
			break;
		}

		HAL_Delay(ICP20100_SHORT_DELAY_MS);
	}

	if (!fifoReady) {
		return ZP_ERROR_TIMEOUT;
	}

	// Step 3: Stop measuring data
	status = waitForModeSync(hi2c);
	if (status != ZP_ERROR_OK) return status;

	status = writeRegisterBlocking(hi2c, ICP20100_REG_MODE_SELECT, 0x00, ICP20100_FIR_WARMUP_I2C_TIMEOUT_MS);
	if (status != ZP_ERROR_OK) return status;

	HAL_Delay(ICP20100_SHORT_DELAY_MS);

	// Step 4: Flush FIFO filter
	status = writeRegisterBlocking(hi2c, ICP20100_FIFO_FILL, ICP20100_FIFO_FILL_FLUSH_BM, ICP20100_FIR_WARMUP_I2C_TIMEOUT_MS);
	if (status != ZP_ERROR_OK) return status;

	// Step 5: Start measurement
	status = waitForModeSync(hi2c);
	if (status != ZP_ERROR_OK) return status;

	// Step 6: Pass data reading to ReadPressureDMA
	return writeRegisterBlocking(hi2c, ICP20100_REG_MODE_SELECT, ICP20100_MODE_SELECT_MEAS_MODE0_CONTINUOUS, ICP20100_FIR_WARMUP_I2C_TIMEOUT_MS);
}

ZP_Error Barometer::readRegister(
    uint16_t memAddress,
    uint8_t * pData,
    uint16_t size) {
	return halToZpError(HAL_I2C_Mem_Read_DMA(hi2c, ICP20100_I2C_ADDR, memAddress, I2C_MEMADD_SIZE_8BIT, pData, size));
}

void Barometer::rxCallback() {
	switch (callbackState) {
		case NOT_STARTED: { // Step 1: Start FIFO fill register read via DMA
			dataFilled = 0;
			if (readRegister(ICP20100_FIFO_FILL, &fifoRegister, 1) == ZP_ERROR_OK) {
				callbackState = FIFO_STARTED;
			} else {
				callbackState = NOT_STARTED;
			}
			break;
		}

		case FIFO_STARTED: { // Step 2: FIFO read complete. If data ready, read pressure/temp burst.
			fifoRegister &= ICP20100_FIFO_FILL_COUNT_MASK;
			if (fifoRegister > 0) {
				if (readRegister(ICP20100_PRESS_DATA_0, pressTempData, ICP20100_PRESS_TEMP_BURST_SIZE) == ZP_ERROR_OK) {
					callbackState = DATA_READ;
				} else {
					callbackState = NOT_STARTED;
				}
			} else {
				// Keep polling FIFO until at least one sample is ready.
				callbackState = NOT_STARTED;
			}
			break;
		}

		case DATA_READ: { // Step 3: Burst read complete. Signal data ready.
			dataFilled = 1;
			callbackState = NOT_STARTED;
			break;
		}

		default: {
			callbackState = NOT_STARTED;
			break;
		}
	}
}

void Barometer::errorCallback() {
	callbackState = NOT_STARTED;
}

ZP_Error Barometer::readData(BaroData_t &data)
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
		data.altitude = (ICP20100_STD_SEA_LEVEL_TEMP_K / ICP20100_TEMP_LAPSE_RATE) *
						 (1.0f - powf(data.pressureKPa / ICP20100_SEA_LEVEL_PRESSURE_KPA, ICP20100_BAROMETRIC_EXPONENT));
		dataFilled = 0;
		rxCallback();
		return ZP_ERROR_OK;
	}

	if (callbackState != NOT_STARTED || HAL_I2C_GetState(hi2c) != HAL_I2C_STATE_READY) {
		return ZP_ERROR_BUSY;
	}
	// Kick off DMA state machine. FIFO polling starts in callback step 1.
	rxCallback();

	// Non-blocking: no data ready yet.
	return ZP_ERROR_NOT_READY;
}

I2C_HandleTypeDef* Barometer::getI2C() {
	return hi2c;
}

// ============================================================================
// Static helper function implementations
// ============================================================================

static inline ZP_Error halToZpError(HAL_StatusTypeDef status) {
    if (status == HAL_OK) {
        return ZP_ERROR_OK;
    } else if (status == HAL_TIMEOUT) {
        return ZP_ERROR_EXT_API | ZP_ERROR_TIMEOUT;
    } else if (status == HAL_BUSY) {
        return ZP_ERROR_EXT_API | ZP_ERROR_BUSY;
    }
    return ZP_ERROR_EXT_API | ZP_ERROR_FAIL;
}

static inline ZP_Error unlockOrLock(I2C_HandleTypeDef *hi2c, bool doLock) {
    return writeRegisterBlocking(hi2c, ICP20100_MASTER_LOCK, doLock ? LOCK_VALUE : UNLOCK_VALUE);
}

static inline ZP_Error readRegisterBlocking(I2C_HandleTypeDef *hi2c, uint16_t memAddress, uint8_t &out, uint32_t timeout) {
    return halToZpError(HAL_I2C_Mem_Read(hi2c, ICP20100_I2C_ADDR, memAddress, I2C_MEMADD_SIZE_8BIT, &out, 1, timeout));
}

static inline ZP_Error writeRegisterBlocking(I2C_HandleTypeDef *hi2c, uint16_t memAddress, uint8_t value, uint32_t timeout) {
    return halToZpError(HAL_I2C_Mem_Write(hi2c, ICP20100_I2C_ADDR, memAddress, I2C_MEMADD_SIZE_8BIT, &value, 1, timeout));
}

static inline ZP_Error waitForOtpStatusClear(I2C_HandleTypeDef *hi2c) {
    const uint32_t startMs = HAL_GetTick();

    do {
        uint8_t otpStatus = ICP20100_OTP_STATUS_BUSY_BM;
        ZP_Error status = readRegisterBlocking(hi2c, ICP20100_OTP_STATUS, otpStatus);
        if (status != ZP_ERROR_OK) {
            return status;
        }

        if ((otpStatus & ICP20100_OTP_STATUS_BUSY_BM) == 0U) {
            return ZP_ERROR_OK;
        }

        HAL_Delay(OTP_STATUS_POLL_INTERVAL_MS);
    } while ((HAL_GetTick() - startMs) < OTP_STATUS_POLL_TIMEOUT_MS);

    return ZP_ERROR_TIMEOUT;
}

static inline ZP_Error writeRegisterWithVerify(I2C_HandleTypeDef *hi2c, uint16_t memAddress, uint8_t value, uint32_t timeout) {
    ZP_Error status = writeRegisterBlocking(hi2c, memAddress, value, timeout);
    if (status != ZP_ERROR_OK) {
        return status;
    }

    uint8_t readBack = 0;
    status = readRegisterBlocking(hi2c, memAddress, readBack, timeout);
    if (status != ZP_ERROR_OK) {
        return status;
    }

    return (readBack == value) ? ZP_ERROR_OK : ZP_ERROR_CRC;
}

static inline ZP_Error waitForModeSync(I2C_HandleTypeDef *hi2c) {
    const uint32_t startMs = HAL_GetTick();

    while ((HAL_GetTick() - startMs) < ICP20100_MODE_SYNC_TIMEOUT_MS) {
        uint8_t deviceStatus = 0;
        ZP_Error status = readRegisterBlocking(hi2c, ICP20100_DEVICE_STATUS, deviceStatus, ICP20100_FIR_WARMUP_I2C_TIMEOUT_MS);
        if (status != ZP_ERROR_OK) {
            return status;
        }

        if (deviceStatus & ICP20100_MODE_SYNC_STATUS_BIT) {
            return ZP_ERROR_OK; // MODE_SELECT is accessible
        }

        HAL_Delay(ICP20100_SHORT_DELAY_MS);
    }

    return ZP_ERROR_TIMEOUT;
}

static inline ZP_Error waitForDeviceReady(I2C_HandleTypeDef *hi2c, uint8_t &version) {
    const uint32_t startMs = HAL_GetTick();

    do {
        if (unlockOrLock(hi2c, false) == ZP_ERROR_OK &&
            unlockOrLock(hi2c, false) == ZP_ERROR_OK &&
            readRegisterBlocking(hi2c, ICP20100_VERSION_REG, version, ICP20100_FIR_WARMUP_I2C_TIMEOUT_MS) == ZP_ERROR_OK &&
            (version == VERSION_B || version == VERSION_A)) {
            return ZP_ERROR_OK;
        }

        HAL_Delay(ICP20100_POWER_ON_POLL_INTERVAL_MS);
    } while ((HAL_GetTick() - startMs) < ICP20100_POWER_ON_TIMEOUT_MS);

    return ZP_ERROR_TIMEOUT;
}

// Selects an OTP address, requests a read, waits for it to finish and reads back the trim byte
static inline ZP_Error readOtpByte(I2C_HandleTypeDef *hi2c, uint8_t otpAddress, uint8_t &out) {
    ZP_Error status = writeRegisterWithVerify(hi2c, ICP20100_OTP_ADDRESS, otpAddress);
    if (status != ZP_ERROR_OK) return status;

    uint8_t command = 0x00;
    status = readRegisterBlocking(hi2c, ICP20100_OTP_COMMAND, command);
    if (status != ZP_ERROR_OK) return status;

    command &= ~ICP20100_OTP_COMMAND_FIELD_MASK;
    command |= ICP20100_OTP_COMMAND_READ_REQUEST;

    status = writeRegisterWithVerify(hi2c, ICP20100_OTP_COMMAND, command);
    if (status != ZP_ERROR_OK) return status;

    status = waitForOtpStatusClear(hi2c);
    if (status != ZP_ERROR_OK) return status;

    return readRegisterBlocking(hi2c, ICP20100_OTP_RDATA, out);
}
