#include "spi_iface.hpp"

SpiDevice::SpiDevice(GPIO_TypeDef *csGPIOPort, uint16_t csGPIOPin, SPI_HandleTypeDef *hspi):
	csGPIOPort(csGPIOPort),
	csGPIOPin(csGPIOPin), 
	hspi(hspi) {}

void SpiDevice::spiCsLow() {
	HAL_GPIO_WritePin(csGPIOPort, csGPIOPin, GPIO_PIN_RESET);
}

void SpiDevice::spiCsHigh() {
	HAL_GPIO_WritePin(csGPIOPort, csGPIOPin, GPIO_PIN_SET);
}

// function to transmit
ZP_Error SpiDevice::spiTransmitData(const uint8_t *txBuf, uint16_t size, uint32_t timeout) {
	// Checking if the pointers will be nullptrs or not
	if (txBuf == nullptr || hspi == nullptr) {
		return ZP_ERROR_NULLPTR;
	}

	// SPI Communication
	spiCsLow();
	HAL_StatusTypeDef status = HAL_SPI_Transmit(hspi, const_cast<uint8_t *>(txBuf), size, timeout);
	spiCsHigh();

	if (status == HAL_OK) {
		return ZP_ERROR_OK;
	}
	return ZP_ERROR_EXT_API | ZP_ERROR_FAIL;
}

// Function to receive data
ZP_Error SpiDevice::spiTransmitReceiveData(const uint8_t *txBuf, uint8_t *rxBuf, uint16_t size, uint32_t timeout) {
	if (txBuf == nullptr || rxBuf == nullptr || hspi == nullptr) {
		return ZP_ERROR_NULLPTR;
	}

	// SPI Communication
	spiCsLow();
	HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(hspi, const_cast<uint8_t *>(txBuf), rxBuf, size, timeout);
	spiCsHigh();

	if (status == HAL_OK) {
		return ZP_ERROR_OK;
	}
	return ZP_ERROR_EXT_API | ZP_ERROR_FAIL;
}

// Function for SPI transmit and receive
ZP_Error SpiDevice::spiTransmitReceiveDataDMA(uint8_t *txBuf, uint8_t *rxBuf, uint16_t size) {
	if (txBuf == nullptr || rxBuf == nullptr || hspi == nullptr) {
		return ZP_ERROR_NULLPTR;
	}

	// SPI Communication
	spiCsLow();
	HAL_StatusTypeDef status = HAL_SPI_TransmitReceive_DMA(hspi, txBuf, rxBuf, size);

	// only turns CS line high if status is not ok
	if (status != HAL_OK) {
		spiCsHigh();
		return ZP_ERROR_EXT_API | ZP_ERROR_FAIL;
	}
	return ZP_ERROR_OK;
}