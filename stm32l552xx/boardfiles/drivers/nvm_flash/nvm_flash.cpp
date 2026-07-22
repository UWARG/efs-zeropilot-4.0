// NVM_Flash.cpp
#include "nvm_flash.hpp"
#include <string.h>

#define WRITE_ENABLE 0x06
#define WRITE_DISABLE 0x04
#define PAGE_PROGRAM   0x02
#define READ 0x03
#define READ_STATUS 0x05
#define FLAG_STATUS 0x70
#define CLEAR_FLAG 0x50
#define SUBSECTOR_ERASE_4K 0x20

NVMFlash::NVMFlash(SPI_HandleTypeDef *spiHandle, GPIO_TypeDef *csPort, uint16_t csPin) :
	spiHandle(spiHandle), csPort(csPort), csPin(csPin) {
	mounted = false;
}

int NVMFlash::format() {
	HAL_StatusTypeDef st = eraseFull();
	if (st != HAL_OK) {
		return -1;
	}

	// Reset in-RAM state
	state.head_idx = 0xFFFFFFFFu;
	state.tail_idx = 0xFFFFFFFFu;
	state.next_idx = 0u;    // next write will go to unit 0
	state.next_id  = 0u;    // first record will have ID 0
	mounted  = false;

	return 0;
}
int NVMFlash::mount() {
	uint32_t min_id  = 0xFFFFFFFFu;
	uint32_t max_id  = 0u;
	uint32_t min_idx = 0xFFFFFFFFu;
	uint32_t max_idx = 0xFFFFFFFFu;

	// Scan every 4 KB unit in the FTL region
	for (uint32_t i = 0; i < FTL_NUM_UNITS; i++) {
		uint32_t base_addr = indexToBaseAddr(i);
		uint32_t hdr_addr  = base_addr;
		uint32_t payload_addr = base_addr + FTL_PAYLOAD_OFFSET;

		FtlRecordHeader hdr;
		memset(&hdr, 0xFF, sizeof(hdr));  // just defensive
		readData(hdr_addr, (uint8_t *)&hdr, (uint16_t)sizeof(hdr));

		// Case 1: completely erased / never used block
		if (hdr.status == FTL_STATUS_EMPTY && hdr.id == 0xFFFFFFFFu) {
			continue;
		}

		// For now, we only care about blocks marked VALID.
		if (hdr.status != FTL_STATUS_VALID) {
			continue;
		}

		// Basic sanity check on length
		if (hdr.length == 0u || hdr.length > FTL_MAX_PAYLOAD) {
			// Length is nonsense; ignore this header.
			continue;
		}

		// (Optional future step: CRC check of payload bytes.
		//  We'll add that once append/write is in place.)
		uint8_t buffer[FTL_MAX_PAYLOAD];
		readData(payload_addr, buffer, hdr.length);

		if (hdr.crc != crc32(buffer, hdr.length)) {
			hdr.status = FTL_STATUS_BAD;
			//TODO: handle corrupted block
			continue;
		}


		// Track oldest (min_id) and newest (max_id) record
		if (hdr.id < min_id) {
			min_id  = hdr.id;
			min_idx = i;
		}
		if (max_idx == 0xFFFFFFFFu || hdr.id > max_id) {
			max_id  = hdr.id;
			max_idx = i;
		}
	}

	if (min_idx == 0xFFFFFFFFu) {
		// No VALID records found at all.
		// This is either a freshly formatted device or fully erased.
		state.head_idx = 0xFFFFFFFFu;
		state.tail_idx = 0xFFFFFFFFu;
		state.next_idx = 0u;  // start writing at unit 0
		state.next_id  = 0u;  // first record will have ID 0
	} else {
		// We found at least one VALID record.
		state.head_idx = max_idx;       // newest
		state.tail_idx = min_idx;       // oldest
		state.next_id  = max_id + 1u;   // ID for the next record

		// Next write goes after 'head', wrapping around
		uint32_t next = state.head_idx + 1u;
		if (next >= FTL_NUM_UNITS) {
			next = 0u;
		}
		state.next_idx = next;
	}

	mounted = true;
	return 0;
}

int NVMFlash::write(AbstractMessage *msg) {
	// ensure that chip is mounted
	if (!mounted) {
		return -1;   // not mounted yet
	}
	if (msg->packed_size() == 0 || msg->packed_size() > FTL_MAX_PAYLOAD) {
		return -2;   // invalid length
	}

	uint32_t idx = state.next_idx;      // which block to use
	uint32_t addr_base = indexToBaseAddr(idx);
	uint32_t id = state.next_id;       // record ID

	uint8_t data[FTL_MAX_PAYLOAD];
	uint16_t len;
	msg->pack(data, len);

	// ======================= Erase the 4 KB block we are going to use
	// TODO: make a low priority task erase in background to save time on writes?
	HAL_StatusTypeDef st = erase4k(addr_base);
	if (st != HAL_OK) {
		return -3;
	}
	// Build header
	FtlRecordHeader hdr;
	memset(&hdr, 0xFF, sizeof(hdr));   // start with entire header as if erased
	hdr.id = id;
	hdr.status = FTL_STATUS_VALID;     // TODO: set this as valid but add crc
	hdr.length = msg->packed_size();
	hdr.crc = crc32((const uint8_t *)data, len);

	// ============================  actually write to the chip

	// program header into first 256-byte page
	st = pageProgram(addr_base, (const uint8_t *) &hdr, (uint16_t) sizeof(hdr));
	if (st != HAL_OK) return -4;

	// program payload (actual data) starting at FTL_PAYLOAD_OFFSET
	const uint8_t *src = (const uint8_t *)data;	// convert void data pointer to uint8_t pointer
	uint32_t remaining = len;	// use this to track what is left, must write page by page
	uint32_t write_addr = addr_base + FTL_PAYLOAD_OFFSET;

	while (remaining > 0) {
		// if remaining bytes is larger than one page, write 256, otherwise if at end of data, write < 256 bytes
		uint16_t chunk = (remaining > FLASH_PAGE_SIZE_256) ? FLASH_PAGE_SIZE_256: (uint16_t) remaining;

		st = pageProgram(write_addr, src, chunk);
		if (st != HAL_OK) {
			return -5;
		}

		src        += chunk;
		write_addr += chunk;
		remaining  -= chunk;
	}

	// =========================================== now must update in-RAM FTL state

	// If this is the first ever record:
	if (state.head_idx == 0xFFFFFFFFu) {
		state.head_idx = idx;
		state.tail_idx = idx;
	} else {
		state.head_idx = idx;

		// If we just overwrote the oldest record, move tail forward
		if (idx == state.tail_idx) {
			uint32_t new_tail = state.tail_idx + 1u;
			if (new_tail >= FTL_NUM_UNITS) {
				new_tail = 0u;
			}
			state.tail_idx = new_tail;
		}
	}

	// Compute next index (circular)
	uint32_t next = idx + 1u;
	if (next >= FTL_NUM_UNITS) {
		next = 0u;
	}
	state.next_idx = next;


	state.next_id = id + 1u; // increment id by 1, wraparound happens naturally

	msg->id = id;

	return 0;
}

int NVMFlash::read(AbstractMessage *msg) {
	if (!mounted) {
		// Ensures the chip is mounted
		return -1;
	}

	uint32_t idx = state.tail_idx;
	FtlRecordHeader header;

	// Search the chip for the matching ID block using the circular buffer
	while (true) {
		memset(&header, 0xFF, sizeof(header));
		readData(indexToBaseAddr(idx), (uint8_t*) &header, (uint16_t) sizeof(header));

		if (header.status == FTL_STATUS_VALID && header.id == msg->id) {
			// Matching ID block found
			break;
		}

		if (idx == state.head_idx) {
			// Looped through all indices, did not find the matching ID block
			return -2;
		}

		idx = (idx + 1) % FTL_NUM_UNITS; // Ensures wrapping
	}

	uint32_t addr_base = indexToBaseAddr(idx);
	uint8_t out[FTL_MAX_PAYLOAD];
	// TODO: Add success/error check to readData
	readData(addr_base + FTL_PAYLOAD_OFFSET, out, header.length);

	msg->unpack(out, header.length);

	return 0;
}

int NVMFlash::erase(AbstractMessage *msg) {
	//wait for write to finish
	HAL_StatusTypeDef wait_result = waitReady(MAX_TIMEOUT);

	if(wait_result != HAL_OK)
	{
		return -1;
	}

	if (!mounted) {
		// Ensures the chip is mounted
		return -2;
	}

	uint32_t idx = state.tail_idx;
	FtlRecordHeader header;

	// Search the chip for the matching ID block using the circular buffer
	while (true) {
		memset(&header, 0xFF, sizeof(header));
		readData(indexToBaseAddr(idx), (uint8_t*) &header, (uint16_t) sizeof(header));

		if (header.status == FTL_STATUS_VALID && header.id == msg->id) {
			// Matching ID block found
			break;
		}

		if (idx == state.head_idx) {
			// Looped through all indices, did not find the matching ID block
			return -3;
		}

		idx = (idx + 1) % FTL_NUM_UNITS; // Ensures wrapping
	}

	uint32_t block_address = indexToBaseAddr(idx);

	//erase the block
	HAL_StatusTypeDef result = erase4k(block_address);

	switch(result){
	//erase succeeded
	case HAL_OK:
		return 0;

	//erase ran into an error
	case HAL_ERROR:
		return -4;

	//erase timed out
	case HAL_TIMEOUT:
		return -5;
	}

	return 0;
}

int NVMFlash::update(AbstractMessage *msg) {
	//check mount
	if (!mounted) {
		return -1;
	}

	//Wait ongoing write
	HAL_StatusTypeDef st = waitReady(MAX_TIMEOUT);
	if (st != HAL_OK) {
		return -2;
	}

	if (msg->packed_size() == 0 || msg->packed_size() > FTL_MAX_PAYLOAD) {
		return -3;   // invalid length
	}

	uint8_t data[FTL_MAX_PAYLOAD];
	uint16_t len;
	msg->pack(data, len);

	int res = read(msg);
	if (res == -2) {
		return -4;
	}

	uint32_t idx = state.next_idx;      // which block to use
	uint32_t addr_base = indexToBaseAddr(idx);
	uint32_t id = msg->id;       // record ID

	// ======================= Erase the 4 KB block we are going to use
	st = erase4k(addr_base);
	if (st != HAL_OK) {
		return -5;
	}

	FtlRecordHeader hdr;
	memset(&hdr, 0xFF, sizeof(hdr));
	hdr.id = id;
	hdr.length = len;
	hdr.status = FTL_STATUS_VALID;
	hdr.crc = crc32((const uint8_t*) data, len);

	st = pageProgram(addr_base, (const uint8_t*) &hdr, (uint16_t) sizeof(hdr));
	if (st != HAL_OK) {
		return -6;
	}

	const uint8_t* src = (const uint8_t*) data;
	uint32_t remaining = len;
	uint32_t write_addr = addr_base + FTL_PAYLOAD_OFFSET;

	while (remaining > 0) {
		uint16_t chunk = remaining > FLASH_PAGE_SIZE_256 ? FLASH_PAGE_SIZE_256 : (uint16_t) remaining;

		st = pageProgram(write_addr, src, chunk);
		if (st != HAL_OK) {
			return -7;
		}

		src 	   += chunk;
		write_addr += chunk;
		remaining  -= chunk;
	}

	state.head_idx = idx;
	if (idx == state.tail_idx) {
		state.tail_idx = (state.tail_idx + 1u) % FTL_NUM_UNITS;
	}
	state.next_idx = (idx + 1u) % FTL_NUM_UNITS;

	res = erase(msg);
	if (res != 0) {
		return -8;
	}

	return 0;
}

void NVMFlash::csLow() {
	HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_RESET);
}
void NVMFlash::csHigh() {
	HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_SET);
}

HAL_StatusTypeDef NVMFlash::tx(const uint8_t *buf, uint16_t len) {
	return HAL_SPI_Transmit(spiHandle, (uint8_t*) buf, len, 1000);
}
HAL_StatusTypeDef NVMFlash::rx(uint8_t *buf, uint16_t len) {
	return HAL_SPI_Receive(spiHandle, buf, len, 1000);
}

/*
 * function to read status register
 * bit0 (LSB) = write in progress: 1 when busy (write, program, or erase in progress), 0 when ready
 * bit1 = write latch, 0 when cleared, 1 when set
 */
uint8_t NVMFlash::readSR() {
    uint8_t sr = 0, cmd = READ_STATUS;
    csLow();
    tx(&cmd);
    rx(&sr, 1);
    csHigh();
    return sr;
}

/*
 * function to read the flag status register
 * bit 8 (MSB) = status bit, 1 means ready, 0 means busy
 * this is different from the read status register, it gives more error codes
 */
uint8_t NVMFlash::readFSR() {
    uint8_t fsr = 0, cmd = FLAG_STATUS;
    csLow();
    tx(&cmd);
    rx(&fsr, 1);
    csHigh();
    return fsr;
}

/*
 * function to clear the flag status register
 * when run into an error and want to try again, good to use this
 */
void NVMFlash::clearFSR() {
    uint8_t cmd = CLEAR_FLAG;
    csLow(); tx(&cmd); csHigh();
}

/*
 * function to enable writes, should call this before every write
 * returns true = write is enabled
 */
bool NVMFlash::writeEnable() {
    uint8_t cmd = WRITE_ENABLE;
    csLow(); tx(&cmd); csHigh();
    // make sure that the write enable latch is set (bit 1 of the status register)
    return (readSR() & 0x02) != 0;
}

/*
 * function to wait a specific time and keep checking the SR register
 * use this to poll status reg after performing a read/write/erase
 */
HAL_StatusTypeDef NVMFlash::waitReady(uint32_t timeout_ms) {
    uint32_t t0 = HAL_GetTick();
    while(1) {
        if ((readSR() & 0x01) == 0) {	// finished when bit 0 of status reg is reset
            uint8_t fsr = readFSR();
            if (fsr & 0x60) { // erase/program error
                clearFSR();
                return HAL_ERROR;
            }
            return HAL_OK;
        }
        if ((HAL_GetTick() - t0) > timeout_ms) return HAL_TIMEOUT;

        HAL_Delay(10);
    }
}

/*
 * function to erase a 4K sector
 * make sure to enable write before calling this
 * the least significant 12 bits of add24 will be ignored (need multiples of 4096)
 */
HAL_StatusTypeDef NVMFlash::erase4k(uint32_t addr24) {
    if (!writeEnable()) return HAL_ERROR;
    uint8_t cmd[4] = { SUBSECTOR_ERASE_4K,
                       (uint8_t)(addr24 >> 16),
                       (uint8_t)(addr24 >> 8),
                       (uint8_t)(addr24) };
    csLow(); tx(cmd, sizeof(cmd)); csHigh();
    return waitReady(400); // should be plenty for 4KB erase
}

HAL_StatusTypeDef NVMFlash::eraseFull() {
	if (!writeEnable()) return HAL_ERROR;

	uint8_t cmd[1] = { 0xC7 };

	csLow(); tx(cmd, sizeof(cmd)); csHigh();
	return waitReady(231000); // Maximum 256 Mb bulk erase time
}

/*
 * function to program data into an array
 * the 24 bit address can be anywhere in the range, but there are page alignment rules
 * no automatic wrap into the next page
 */
HAL_StatusTypeDef NVMFlash::pageProgram(uint32_t addr24, const uint8_t *data, uint16_t len) {
    if (len == 0 || len > 256) return HAL_ERROR;
    if (!writeEnable()) return HAL_ERROR;

    // check to make sure alignment is correct (write at the start of a page)
    if (((addr24 & 0xFF) + len) > 256 ) return HAL_ERROR;

    uint8_t hdr[4] = { PAGE_PROGRAM,
                       (uint8_t)(addr24 >> 16),
                       (uint8_t)(addr24 >> 8),
                       (uint8_t)(addr24) };
    csLow();
    tx(hdr, sizeof(hdr));
    tx(data, len);
    csHigh();
    return waitReady(10); // page program typically <1ms; 10ms guard
}

/*
 * function to read data
 * addr24 is where the data will start from, and increment
 */
void NVMFlash::readData(uint32_t addr24, uint8_t *out, uint16_t len) {
    uint8_t hdr[4] = { READ,
                       (uint8_t)(addr24 >> 16),
                       (uint8_t)(addr24 >> 8),
                       (uint8_t)(addr24) };
    csLow();
    tx(hdr, sizeof(hdr));   // send cmd+addr
    rx(out, len);           // then clock out data
    csHigh();
}

/*
 * - uint8_t *data: data array
 * - int len: the length of input data array
 * returns LSB-first (reflected) CRC result.
 */
uint32_t NVMFlash::crc32(const uint8_t *data, uint32_t len) {
	//append 8 zero bits by shifting to the left
	uint32_t crc = 0xFFFFFFFF;

    for (int i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1)
                crc = (crc >> 1) ^ CRC_POLY;
            else
                crc >>= 1;
        }
    }

    return crc ^ 0xFFFFFFFF;
}

inline uint32_t NVMFlash::indexToBaseAddr(uint32_t unit_index) {
    return (unit_index * FTL_UNIT_SIZE);  // FTL_DATA_BASE is 0, so this is fine
}
