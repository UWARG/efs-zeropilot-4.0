// NVM_Flash.hpp
#pragma once

#include "stm32l5xx_hal.h"
#include "nvm_flash_message.hpp"

class NVMFlash {
public:
	NVMFlash(SPI_HandleTypeDef *spiHandle, GPIO_TypeDef *csPort, uint16_t csPin);

	int format();
	int mount();

	int write(AbstractMessage *msg);
	int read(AbstractMessage *msg);
	int erase(AbstractMessage *msg);
	int update(AbstractMessage *msg);

private:
	SPI_HandleTypeDef *spiHandle;
	GPIO_TypeDef *csPort;
	uint16_t csPin;

	bool mounted;

	static constexpr uint32_t FLASH_TOTAL_SIZE_BYTES = 32u * 1024u * 1024u;  // 32 MB total = 33 554 432 bytes
	static constexpr uint32_t FLASH_SECTOR_SIZE_4K = 4096u;   // erase size in bytes
	static constexpr uint32_t FLASH_PAGE_SIZE_256 = 256u;    // page program size in bytes

	// FTL geometry
	static constexpr uint32_t FTL_UNIT_SIZE = FLASH_SECTOR_SIZE_4K;   // 4 KB per unit
	static constexpr uint32_t FTL_HEADER_PAGE_SIZE = FLASH_PAGE_SIZE_256;		// First page (256 B) of each unit is the header page
	static constexpr uint32_t FTL_PAYLOAD_OFFSET = FTL_HEADER_PAGE_SIZE;	// Payload starts at the beginning of the second page
	static constexpr uint32_t FTL_MAX_PAYLOAD = FTL_UNIT_SIZE - FTL_HEADER_PAGE_SIZE;	// Max payload per unit: everything after the first 256 B page
	static constexpr uint32_t FTL_NUM_UNITS = FLASH_TOTAL_SIZE_BYTES / FTL_UNIT_SIZE;	 // Number of usable 4 KB units in the flash

	static constexpr uint32_t FTL_INVALID_PAGE = 0xFFFFFFFFu;

	static constexpr uint32_t CRC_POLY = 0xEDB88320;

	static constexpr uint16_t MAX_TIMEOUT = 6000u;

	struct __attribute__((packed)) FtlRecordHeader {	// use packed so that no padding is inserted by the compiler
	    uint32_t id;        // Monotonically increasing record ID (for ordering).
	    uint16_t status;    // Lifecycle status (see FTL_STATUS_* below).
	    uint16_t length;    // Payload length in bytes (0 < length <= FTL_MAX_PAYLOAD).
	    uint32_t crc;       // CRC32 over the payload bytes.
	    uint8_t  reserved[8]; // Reserved for future use (timestamps, logical addr, etc.).
	};

	// =========== status values
	static constexpr uint16_t FTL_STATUS_EMPTY = 0xFFFFu;  // Erased / never written (all 0xFF in header).
	static constexpr uint16_t FTL_STATUS_VALID = 0x3FFFu;  // Fully written and CRC-verified at write time.
	static constexpr uint16_t FTL_STATUS_STALE = 0x1FFFu;  // Logically obsolete (superseded by newer data).
	static constexpr uint16_t FTL_STATUS_BAD = 0x0FFFu;  // Known-bad record (CRC fail, header nonsense, etc.).

	struct FtlState {
	    uint32_t head_idx;
	    uint32_t tail_idx;
	    uint32_t next_idx;
	    uint32_t next_id;
	    bool     mounted;
	};

	FtlState state = {
			0xFFFFFFFFu,
			0xFFFFFFFFu,
			0u,
			0u,
			false
	};

	void csLow();
	void csHigh();

	HAL_StatusTypeDef tx(const uint8_t *buf, uint16_t len = 1);
	HAL_StatusTypeDef rx(uint8_t *buf, uint16_t len);

	uint8_t readSR(); // Read Status Register
	uint8_t readFSR(); // Read Flag Status Register
	void clearFSR();

	bool writeEnable();

	HAL_StatusTypeDef waitReady(uint32_t timeout_ms = 0);

	HAL_StatusTypeDef erase4k(uint32_t addr24);
	HAL_StatusTypeDef eraseFull();
	HAL_StatusTypeDef pageProgram(uint32_t addr24, const uint8_t *data, uint16_t len);
	void readData(uint32_t addr24, uint8_t *out, uint16_t len);

	static uint32_t crc32(const uint8_t *data, uint32_t len);
	static inline uint32_t indexToBaseAddr(uint32_t unit_index);
};
