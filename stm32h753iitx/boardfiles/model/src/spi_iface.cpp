#include "spi_iface.hpp"

ISpi::ISpi(GPIO_TypeDef *csGPIOPort, uint16_t csGPIOPin, SPI_HandleTypeDef *hspi):
	csGPIOPort(csGPIOPort),
	csGPIOPin(csGPIOPin), 
	hspi(hspi) {}

ZP_Error ISpi::csLow() {
	if (csGPIOPort == nullptr) return ZP_ERROR_NULLPTR; 
	if (csGPIOPin == 0) return ZP_ERROR_INVALID_ARG;
	HAL_GPIO_WritePin(csGPIOPort, csGPIOPin, GPIO_PIN_RESET);
	return ZP_ERROR_OK;
}

ZP_Error ISpi::csHigh() {
	if (csGPIOPort == nullptr) return ZP_ERROR_NULLPTR;
	if (csGPIOPin == 0) return ZP_ERROR_INVALID_ARG;
	HAL_GPIO_WritePin(csGPIOPort, csGPIOPin, GPIO_PIN_SET);
	return ZP_ERROR_OK;
}

ZP_Error ISpi::transmit(uint8_t *txBuf, uint16_t size, uint32_t timeout) {
	if (txBuf == nullptr || hspi == nullptr) return ZP_ERROR_NULLPTR;

	ZP_Error csErr = csLow();
	if (csErr != ZP_ERROR_OK) return csErr;
	HAL_StatusTypeDef status = HAL_SPI_Transmit(hspi, txBuf, size, timeout);
	csErr = csHigh();
	if (csErr != ZP_ERROR_OK) return csErr;

	return halToZpError(status, hspi->ErrorCode);
}

ZP_Error ISpi::transceive(uint8_t *txBuf, uint8_t *rxBuf, uint16_t size, uint32_t timeout) {
	if (txBuf == nullptr || rxBuf == nullptr || hspi == nullptr) return ZP_ERROR_NULLPTR;

	ZP_Error csErr = csLow();
	if (csErr != ZP_ERROR_OK) return csErr;
	HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(hspi, txBuf, rxBuf, size, timeout);
	csErr = csHigh();
	if (csErr != ZP_ERROR_OK) return csErr;

	return halToZpError(status, hspi->ErrorCode);
}

ZP_Error ISpi::transceiveDMA(uint8_t *txBuf, uint8_t *rxBuf, uint16_t size) {
	if (txBuf == nullptr || rxBuf == nullptr || hspi == nullptr) return ZP_ERROR_NULLPTR;

	ZP_Error csErr = csLow();
	if (csErr != ZP_ERROR_OK) return csErr;
	HAL_StatusTypeDef status = HAL_SPI_TransmitReceive_DMA(hspi, txBuf, rxBuf, size);
	if (status != HAL_OK) {
		csErr = csHigh();
		if (csErr != ZP_ERROR_OK) return csErr;
	}

	return halToZpError(status, hspi->ErrorCode);
}