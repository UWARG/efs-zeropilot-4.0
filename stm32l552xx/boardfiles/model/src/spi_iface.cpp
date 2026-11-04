#include "spi_iface.hpp"

ISpi::ISpi(GPIO_TypeDef *csGPIOPort, uint16_t csGPIOPin, SPI_HandleTypeDef *hspi):
	csGPIOPort(csGPIOPort),
	csGPIOPin(csGPIOPin), 
	hspi(hspi) {}

void ISpi::csLow() {
	HAL_GPIO_WritePin(csGPIOPort, csGPIOPin, GPIO_PIN_RESET);
}

void ISpi::csHigh() {
	HAL_GPIO_WritePin(csGPIOPort, csGPIOPin, GPIO_PIN_SET);
}

ZP_Error ISpi::transmit(uint8_t *txBuf, uint16_t size, uint32_t timeout) {
	if (txBuf == nullptr || hspi == nullptr) { return ZP_ERROR_NULLPTR; }

	csLow();
	HAL_StatusTypeDef status = HAL_SPI_Transmit(hspi, txBuf, size, timeout);
	csHigh();

	return halToZpError(status, hspi->ErrorCode);
}

ZP_Error ISpi::transceive(uint8_t *txBuf, uint8_t *rxBuf, uint16_t size, uint32_t timeout) {
	if (txBuf == nullptr || rxBuf == nullptr || hspi == nullptr) { return ZP_ERROR_NULLPTR; }

	csLow();
	HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(hspi, txBuf, rxBuf, size, timeout);
	csHigh();

	return halToZpError(status, hspi->ErrorCode);
}

ZP_Error ISpi::transceiveDMA(uint8_t *txBuf, uint8_t *rxBuf, uint16_t size) {
	if (txBuf == nullptr || rxBuf == nullptr || hspi == nullptr) { return ZP_ERROR_NULLPTR; }

	csLow();
	HAL_StatusTypeDef status = HAL_SPI_TransmitReceive_DMA(hspi, txBuf, rxBuf, size);
	if (status != HAL_OK) { csHigh(); }

	return halToZpError(status, hspi->ErrorCode);
}