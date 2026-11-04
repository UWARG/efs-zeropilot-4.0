#include "led_is31fl3195.hpp"

const uint8_t IS31FL3195_MAGIC_VALUE = 0xC5;

IS31FL3195_Channel IS31FL3195_ALL_CHANNELS[4] = { { // CH1
		.REG_CURRENT_LEVEL = 0x10, }, { // CH2
		.REG_CURRENT_LEVEL = 0x21, }, { // CH3
		.REG_CURRENT_LEVEL = 0x32, }, { // CH4
		.REG_CURRENT_LEVEL = 0x40, }, };

typedef enum {
	IS31FL3195_REG_PRODUCT_ID = 0x00,
	IS31FL3195_REG_SHUTDOWN_CONTROL = 0x01,
	IS31FL3195_REG_OPERATING_CONFIGURE = 0x02,
	IS31FL3195_COLOR_UPDATE = 0x50,

	IS31FL3195_REG_RESET = 0x5F,
} IS31FL3195_Special_Register;

IS31FL3195::IS31FL3195(I2C_HandleTypeDef *hi2c, uint8_t address) :
		hi2c(hi2c), address(address) {
}

ZP_Error IS31FL3195::begin() {
	ZP_Error status;

	IS31FL3195_ConfReg conf_reg = { .mode_1 = IS31FL3195_MODE_CURRENT_LEVEL };
	status = this->setConfigure(conf_reg);
	if (status != ZP_ERROR_OK)
		return status;

	IS31FL3195_ShutdownCtrl shutdown_ctrl_reg = {
			.softwareShutdownEnable = 1,
			.sleepModeEnable = 0,
			.chargePumpPowerUpMode = 0,

			.enable_1 = 1,
			.enable_2 = 1,
			.enable_3 = 1,
			.enable_4 = 1,
	};
	status = this->setShutdownControl(shutdown_ctrl_reg);
	if (status != ZP_ERROR_OK)
		return status;

	return ZP_ERROR_OK;
}

ZP_Error IS31FL3195::registerWrite(IS31FL3195_Register reg,
		uint8_t val) {
	// as per datasheet
	uint8_t pData[2] = { static_cast<uint8_t>(reg), val };
	HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(hi2c, address, pData, sizeof(pData),
			HAL_MAX_DELAY);
	if(status == HAL_OK)
		return ZP_ERROR_OK;
	else
		return ZP_ERROR_FAIL | ZP_ERROR_EXT_API;
}

ZP_Error IS31FL3195::reset() {
	return this->registerWrite(IS31FL3195_REG_RESET, IS31FL3195_MAGIC_VALUE);
}

ZP_Error IS31FL3195::setConfigure(IS31FL3195_ConfReg conf) {
	return this->registerWrite(IS31FL3195_REG_OPERATING_CONFIGURE, *(uint8_t*) &conf);
}

ZP_Error IS31FL3195::setShutdownControl(IS31FL3195_ShutdownCtrl conf) {
	return this->registerWrite(IS31FL3195_REG_SHUTDOWN_CONTROL, *(uint8_t*) &conf);
}

ZP_Error IS31FL3195::writeCurrentLevel(IS31FL3195_Channel &channel,
		IS31FL3195_CurrentLevel level) {
	return this->registerWrite(channel.REG_CURRENT_LEVEL, level);
}

ZP_Error IS31FL3195::flush() {
	return this->registerWrite(IS31FL3195_COLOR_UPDATE, IS31FL3195_MAGIC_VALUE);
}

ZP_Error IS31FL3195::setSolidColor(IS31FL3195_CurrentLevel rgbw[4]) {
	ZP_Error status;

	if(rgbw == nullptr) return ZP_ERROR_NULLPTR;

	for (size_t i = 0; i < 4; i++) {
		status = this->writeCurrentLevel(IS31FL3195_ALL_CHANNELS[i], rgbw[i]);
		if (status != ZP_ERROR_OK)
			return ZP_ERROR_OK;
	}

	status = this->flush();
	if (status != ZP_ERROR_OK)
		return status;

	return ZP_ERROR_OK;
}

IS31FL3195::~IS31FL3195() {
}

