#include "barometer.hpp"

extern volatile uint32_t g_memrx_start_fail;
extern volatile uint32_t g_i2c_err;

Barometer::Barometer(I2C_HandleTypeDef *hi2c) :
	hi2c(hi2c), callBackCount(0), FIFO_REGISTER(0) {}

bool ICP20100::initiateBarometer()
{
	uint32_t err;

	//Step 1: Power on ASIC

	//Step 2: Write to lock register twice to get access to main registers and initiate communication w/ I2C
	uint8_t unlock = ICP20100_MASTER_UNLOCK_KEY;
	if(HAL_I2C_Mem_Write(hi2c, ICP20100_I2C_ADDR, ICP20100_MASTER_LOCK, I2C_MEMADD_SIZE_8BIT, &unlock, 1, HAL_MAX_DELAY) != HAL_OK){
		err = HAL_I2C_GetError(hi2c);
		return false;
	}

	if(HAL_I2C_Mem_Write(hi2c, ICP20100_I2C_ADDR, ICP20100_MASTER_LOCK, I2C_MEMADD_SIZE_8BIT, &unlock,1, HAL_MAX_DELAY) != HAL_OK){
		err = HAL_I2C_GetError(hi2c);
		return false;
	}

	// Step 3: Read from the version register,
	uint8_t version = 0x00;
	if(HAL_I2C_Mem_Read(hi2c, ICP20100_I2C_ADDR, ICP20100_VERSION_REG, I2C_MEMADD_SIZE_8BIT, &version, 1, HAL_MAX_DELAY) != HAL_OK){
		err = HAL_I2C_GetError(hi2c);
		return false;
	}

	if(version == 0xB2){ // Initialization done if version B
		return false;
	}

	// Step 4: Check boot up status from OTP_Status2 register. Check specifically bit 0.
	uint8_t boot_status = 0x00;
	if(HAL_I2C_Mem_Read(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_STATUS2, I2C_MEMADD_SIZE_8BIT, &boot_status, 1, HAL_MAX_DELAY) != HAL_OK){
		err = HAL_I2C_GetError(hi2c);
		return false;
	}

	// Mask boot status register to only read the 0th bit
	boot_status &= (0x01);

	if(boot_status == 1){ // Initialization done, barometer did not go through power cycle.
		return false;
	}

	// Step 5: Bring ASIC into power mode to get access to main registers
	// Set the 3rd bit of the mode_select register to 1.
	uint8_t mode_select = 0x00;
	if(HAL_I2C_Mem_Read(hi2c, ICP20100_I2C_ADDR, ICP20100_REG_MODE_SELECT, I2C_MEMADD_SIZE_8BIT, &mode_select, 1, HAL_MAX_DELAY) != HAL_OK){
		err = HAL_I2C_GetError(hi2c);
		return false;
	}

	mode_select |= (0x04); // Read previous register and toggle the 3rd bit to preserve previous bits

	if(HAL_I2C_Mem_Write(hi2c, ICP20100_I2C_ADDR, ICP20100_REG_MODE_SELECT, I2C_MEMADD_SIZE_8BIT, &mode_select, 1, HAL_MAX_DELAY) != HAL_OK){
		err = HAL_I2C_GetError(hi2c);
		return false;
	}

	HAL_Delay(4); // blocking delay 4, as required by data sheet

	// Step 6: Unlock main registers by setting the Master_Lock register to 0x1f

	if(HAL_I2C_Mem_Write(hi2c, ICP20100_I2C_ADDR, ICP20100_MASTER_LOCK, I2C_MEMADD_SIZE_8BIT, &unlock, 1, HAL_MAX_DELAY) != HAL_OK){
			err = HAL_I2C_GetError(hi2c);
			return false;
	}


	//Step 7: Enable OTP and write switch by setting the config1 register's bits 0 and 1 to 1.
	uint8_t otp_config = 0x00;
	if(HAL_I2C_Mem_Read(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_CONFIG_1, I2C_MEMADD_SIZE_8BIT, &otp_config, 1, HAL_MAX_DELAY) != HAL_OK){
		err = HAL_I2C_GetError(hi2c); // Read register to preserve bits before setting
		return false;
	}

	otp_config |= (0x03); // Sets bits 011

	if(HAL_I2C_Mem_Write(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_CONFIG_1, I2C_MEMADD_SIZE_8BIT, &otp_config, 1, HAL_MAX_DELAY) != HAL_OK){
		err = HAL_I2C_GetError(hi2c);
		return false;
	}

	HAL_Delay(1); // should be wait 10 microseconds

	//Step 8: Toggle the OTP_DBG2 register bit 8 (reset bit)
	uint8_t reset = 0x00;

	if(HAL_I2C_Mem_Read(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_DBG2, I2C_MEMADD_SIZE_8BIT, &reset, 1, HAL_MAX_DELAY) != HAL_OK){
		err = HAL_I2C_GetError(hi2c);
		return false;
	}

	reset |= (0x80);

	if(HAL_I2C_Mem_Write(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_DBG2, I2C_MEMADD_SIZE_8BIT, &reset, 1, HAL_MAX_DELAY) != HAL_OK){
		err = HAL_I2C_GetError(hi2c);
		return false;
	}

	HAL_Delay(1);

	reset &= ~(0x80);

	if(HAL_I2C_Mem_Write(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_DBG2, I2C_MEMADD_SIZE_8BIT, &reset, 1, HAL_MAX_DELAY) != HAL_OK){
		err = HAL_I2C_GetError(hi2c);
		return false;
	}

	HAL_Delay(1);

	// STEP 9: Program redundant read
	uint8_t redundant_read = 0x00;

	redundant_read = 0x04;
	if(HAL_I2C_Mem_Write(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_MRA_LSB, I2C_MEMADD_SIZE_8BIT, &redundant_read, 1, HAL_MAX_DELAY) != HAL_OK){ err = HAL_I2C_GetError(hi2c); return false; }
	if(HAL_I2C_Mem_Write(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_MRA_MSB, I2C_MEMADD_SIZE_8BIT, &redundant_read, 1, HAL_MAX_DELAY) != HAL_OK){ err = HAL_I2C_GetError(hi2c); return false; }

	redundant_read = 0x21;
	if(HAL_I2C_Mem_Write(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_MRB_LSB, I2C_MEMADD_SIZE_8BIT, &redundant_read, 1, HAL_MAX_DELAY) != HAL_OK){ err = HAL_I2C_GetError(hi2c); return false; }

	redundant_read = 0x20;
	if(HAL_I2C_Mem_Write(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_MRB_MSB, I2C_MEMADD_SIZE_8BIT, &redundant_read, 1, HAL_MAX_DELAY) != HAL_OK){ err = HAL_I2C_GetError(hi2c); return false; }

	redundant_read = 0x10;
	if(HAL_I2C_Mem_Write(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_MR_LSB, I2C_MEMADD_SIZE_8BIT, &redundant_read, 1, HAL_MAX_DELAY) != HAL_OK){ err = HAL_I2C_GetError(hi2c); return false; }

	redundant_read = 0x80;
	if(HAL_I2C_Mem_Write(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_MR_MSB, I2C_MEMADD_SIZE_8BIT, &redundant_read, 1, HAL_MAX_DELAY) != HAL_OK){ err = HAL_I2C_GetError(hi2c); return false; }

	// STEP 10: Write address content and read command
	uint8_t command_address = 0x00;
	command_address = 0xF8;
	if(HAL_I2C_Mem_Write(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_ADDRESS, I2C_MEMADD_SIZE_8BIT, &command_address, 1, HAL_MAX_DELAY) != HAL_OK){ err = HAL_I2C_GetError(hi2c); return false; }

	command_address = 0x00; //X0010000
	if(HAL_I2C_Mem_Read(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_COMMAND, I2C_MEMADD_SIZE_8BIT, &command_address, 1, HAL_MAX_DELAY) != HAL_OK){ err = HAL_I2C_GetError(hi2c); return false; }

	command_address &= ~(0x7F);
	command_address |= (0x10);

	if(HAL_I2C_Mem_Write(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_COMMAND, I2C_MEMADD_SIZE_8BIT, &command_address, 1, HAL_MAX_DELAY) != HAL_OK){ err = HAL_I2C_GetError(hi2c); return false; }

	// STEP 11: Wait for OTP read to finish
	uint8_t status = 1;
	int timeout = 1000;
	while((status & 0x01) && timeout--)
	{
	    if(HAL_I2C_Mem_Read(hi2c, ICP20100_I2C_ADDR,
	        ICP20100_OTP_STATUS,
	        I2C_MEMADD_SIZE_8BIT,
	        &status,
	        1,
	        HAL_MAX_DELAY) != HAL_OK)
	    {
	        err = HAL_I2C_GetError(hi2c);
	        return false;
	    }
	}

	if(timeout <= 0)
	{
	    err = HAL_TIMEOUT;
	    return false;
	}

	// STEP 12: Read offset from the OTP_RDATA register
	uint8_t offset = 0x0000;
	if(HAL_I2C_Mem_Read(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_RDATA, I2C_MEMADD_SIZE_8BIT, &offset, 1, HAL_MAX_DELAY) != HAL_OK){ err = HAL_I2C_GetError(hi2c); return false; }

	// STEP 13: Write next address

	command_address = 0xF9;
	if(HAL_I2C_Mem_Write(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_ADDRESS, I2C_MEMADD_SIZE_8BIT, &command_address, 1, HAL_MAX_DELAY) != HAL_OK){ err = HAL_I2C_GetError(hi2c); return false; }

	command_address = 0x00;
	if(HAL_I2C_Mem_Read(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_COMMAND, I2C_MEMADD_SIZE_8BIT, &command_address, 1, HAL_MAX_DELAY) != HAL_OK){ err = HAL_I2C_GetError(hi2c); return false; }

	command_address &= ~(0x7F);
	command_address |= (0x10);

	if(HAL_I2C_Mem_Write(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_COMMAND, I2C_MEMADD_SIZE_8BIT, &command_address, 1, HAL_MAX_DELAY) != HAL_OK){ err = HAL_I2C_GetError(hi2c); return false; }

	// STEP 14: Wait for OTP read to finish

	status = 1;
	timeout = 1000;
	while((status & 0x01) && timeout--)
		{
		    if(HAL_I2C_Mem_Read(hi2c, ICP20100_I2C_ADDR,
		        ICP20100_OTP_STATUS,
		        I2C_MEMADD_SIZE_8BIT,
		        &status,
		        1,
		        HAL_MAX_DELAY) != HAL_OK)
		    {
		        err = HAL_I2C_GetError(hi2c);
		        return false;
		    }
		}

	if(timeout <= 0)
	{
	    err = HAL_TIMEOUT;
	    return false;
	}

	// STEP 15: Read gain from OTP_RDATA register
	uint8_t gain = 0x0000;
	if(HAL_I2C_Mem_Read(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_RDATA, I2C_MEMADD_SIZE_8BIT, &gain, 1, HAL_MAX_DELAY) != HAL_OK){ err = HAL_I2C_GetError(hi2c); return false; }

	// Step 16: Write next address content

	command_address = 0xFA;
	if(HAL_I2C_Mem_Write(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_ADDRESS, I2C_MEMADD_SIZE_8BIT, &command_address, 1, HAL_MAX_DELAY) != HAL_OK){ err = HAL_I2C_GetError(hi2c); return false; }

	command_address = 0x00; //X0010000
	if(HAL_I2C_Mem_Read(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_COMMAND, I2C_MEMADD_SIZE_8BIT, &command_address, 1, HAL_MAX_DELAY) != HAL_OK){ err = HAL_I2C_GetError(hi2c); return false; }

	command_address &= ~(0x7F);
	command_address |= (0x10);

	if(HAL_I2C_Mem_Write(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_COMMAND, I2C_MEMADD_SIZE_8BIT, &command_address, 1, HAL_MAX_DELAY) != HAL_OK){ err = HAL_I2C_GetError(hi2c); return false; }

	// STEP 17: Wait for OTP read to finish
	status = 1;
	timeout = 1000;
	while((status & 0x01) && timeout--)
			{
			    if(HAL_I2C_Mem_Read(hi2c, ICP20100_I2C_ADDR,
			        ICP20100_OTP_STATUS,
			        I2C_MEMADD_SIZE_8BIT,
			        &status,
			        1,
			        HAL_MAX_DELAY) != HAL_OK)
			    {
			        err = HAL_I2C_GetError(hi2c);
			        return false;
			    }
			}

	if(timeout <= 0)
	{
		err = HAL_TIMEOUT;
		return false;
	}

	// STEP 18: Read HFosc
	uint8_t HFosc = 0x00;
	if(HAL_I2C_Mem_Read(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_RDATA, I2C_MEMADD_SIZE_8BIT, &HFosc, 1, HAL_MAX_DELAY) != HAL_OK){ err = HAL_I2C_GetError(hi2c); return false; }

	// STEP 19: Disable OTP

	if(HAL_I2C_Mem_Read(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_CONFIG_1, I2C_MEMADD_SIZE_8BIT, &otp_config, 1, HAL_MAX_DELAY) != HAL_OK){ err = HAL_I2C_GetError(hi2c); return false; }

	otp_config &= ~(0x03);

	if(HAL_I2C_Mem_Write(hi2c,ICP20100_I2C_ADDR, ICP20100_OTP_CONFIG_1, I2C_MEMADD_SIZE_8BIT, &otp_config, 1, HAL_MAX_DELAY) != HAL_OK){ err = HAL_I2C_GetError(hi2c); return false; }

	HAL_Delay(1); // should be wait 10 microseconds

	// STEP 20: Write offset to main registers
	uint8_t trim_reg;

	if(HAL_I2C_Mem_Read(hi2c, ICP20100_I2C_ADDR, ICP20100_TRIM1_MSB, I2C_MEMADD_SIZE_8BIT, &trim_reg, 1, HAL_MAX_DELAY) != HAL_OK){ err = HAL_I2C_GetError(hi2c); return false; }

	// Clear the 6-bit PEFE_OFFSET_TRIM field (bits 5:0)
	trim_reg &= ~0x3F;

	uint8_t offset_low = offset & 0x3F;
	trim_reg |= offset_low;

	// Write back
	if(HAL_I2C_Mem_Write(hi2c, ICP20100_I2C_ADDR, ICP20100_TRIM1_MSB,
	                  I2C_MEMADD_SIZE_8BIT, &trim_reg, 1, HAL_MAX_DELAY) != HAL_OK){ err = HAL_I2C_GetError(hi2c); return false; }

	// STEP 21: Write gain to main registers
	uint8_t Rdata = 0x00;
	if(HAL_I2C_Mem_Read(hi2c, ICP20100_I2C_ADDR, ICP20100_TRIM2_MSB,
					 I2C_MEMADD_SIZE_8BIT, &Rdata, 1, HAL_MAX_DELAY) != HAL_OK){ err = HAL_I2C_GetError(hi2c); return false; }

	Rdata &= ~(0b01110000);
	gain &= (0x07);
	Rdata |= (gain << 4);

	if(HAL_I2C_Mem_Write(hi2c, ICP20100_I2C_ADDR, ICP20100_TRIM2_MSB,
		                  I2C_MEMADD_SIZE_8BIT, &Rdata, 1, HAL_MAX_DELAY) != HAL_OK){ err = HAL_I2C_GetError(hi2c); return false; }

	// STEP 22: Write HFosc trim value to main registers
	if(HAL_I2C_Mem_Write(hi2c, ICP20100_I2C_ADDR, ICP20100_TRIM2_LSB,
			                  I2C_MEMADD_SIZE_8BIT, &HFosc, 1, HAL_MAX_DELAY) != HAL_OK){ err = HAL_I2C_GetError(hi2c); return false; }

	// STEP 23: Lock main registers
	uint8_t lock = 0x00;
	if(HAL_I2C_Mem_Write(hi2c, ICP20100_I2C_ADDR, ICP20100_MASTER_LOCK, I2C_MEMADD_SIZE_8BIT, &lock, 1, HAL_MAX_DELAY) != HAL_OK){ err = HAL_I2C_GetError(hi2c); return false; }

	 // STEP 24: Move to standby
	uint8_t power_mode = 0;

	if(HAL_I2C_Mem_Read(hi2c,
		                  ICP20100_I2C_ADDR,
		                  ICP20100_REG_MODE_SELECT,
		                  I2C_MEMADD_SIZE_8BIT,
		                  &power_mode,
		                  1,
		                  HAL_MAX_DELAY) != HAL_OK){ err = HAL_I2C_GetError(hi2c); return false; }

	power_mode &= ~(0x04);

	if(HAL_I2C_Mem_Write(hi2c,
	                  ICP20100_I2C_ADDR,
	                  ICP20100_REG_MODE_SELECT,
	                  I2C_MEMADD_SIZE_8BIT,
	                  &power_mode,
	                  1,
	                  HAL_MAX_DELAY) != HAL_OK){ err = HAL_I2C_GetError(hi2c); return false; }

	// STEP 25: Check boot up status to 1, avoid reintialization

	uint8_t boot_config = ICP20100_OTP_STATUS2_BOOTUP;

	if(HAL_I2C_Mem_Write(hi2c, ICP20100_I2C_ADDR, ICP20100_OTP_STATUS2, I2C_MEMADD_SIZE_8BIT, &boot_config, 1, HAL_MAX_DELAY) != HAL_OK){ err = HAL_I2C_GetError(hi2c); return false; }

	return true;
}

bool ICP20100::firWarmupPoll()
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