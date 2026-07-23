#include "gemini_defines.hpp"
#include "gemini.hpp"
#include <cstring>
#include "mavlink.h"

GeminiMavlink* GeminiMavlink::instance = nullptr; // global instance

GeminiMavlink::GeminiMavlink(UART_HandleTypeDef* huart): huart(huart), readIndex(0), writeIndex(0) {
    // Create only one instance of GeminiMavlink
    instance = this;
}

GeminiMavlink::~GeminiMavlink() {
    instance = nullptr;
}

void GeminiMavlink::init() {
    if(huart) {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, rfdRxBuffer, MAVLINK_MAX_PACKET_SIZE);
    }
}

void GeminiMavlink::transmit(const uint8_t* data, uint16_t size) {
    if(huart) {
        HAL_UART_Transmit_DMA(huart, data, size);
    }
}


void GeminiMavlink::receiveCallback(uint16_t writeIdx) {
    if (HAL_UARTEx_GetRxEventType(huart) == HAL_UART_RXEVENT_HT) {
		return;
	}

    writeIndex = writeIdx % BUFFER_SIZE;

	uint16_t transferSize = getRXTransferSize(writeIndex);
	currentSize += transferSize;

    if (currentSize > (BUFFER_SIZE - 1)) {
        readIndex += currentSize - (BUFFER_SIZE - 1);
        readIndex %= BUFFER_SIZE;
        currentSize = BUFFER_SIZE - 1;
    }

	lastIdx = writeIdx;
}


uint16_t GeminiMavlink::receive(uint8_t* buffer, uint16_t bufferSize) {
    for(uint16_t i = 0; i < bufferSize; i++) {
        if(readIndex == writeIndex) {
            return i;
        }
        buffer[i] = rfdRxBuffer[readIndex];
        readIndex++;
        if(readIndex >= MAVLINK_MAX_PACKET_SIZE) {
            readIndex = 0;
        }
    }
    return bufferSize;
}




void GeminiMavlink::startDMA() {
    // start circular DMA
    HAL_UARTEx_ReceiveToIdle_DMA(huart, crsfRxBuffer, CRSF_BYTE_COUNT);
}



// Polynomial used in CRSF: 0xD5
static uint8_t crsf_crc8(const uint8_t *data, uint8_t len) {
    uint8_t crc = 0;
    while (len--) {
        crc ^= *data++;
        for (int i = 0; i < 8; i++) {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0xD5;
            else
                crc <<= 1;
        }
    }
    return crc;
}




void GeminiMavlink::parse() {

    uint8_t *buf = crsfRxBuffer;

    // Validate sync byte and frame type
   if (buf[0] != CRSF_SYNC_BYTE || buf[2] != CRSF_FRAMETYPE_RC_CHANNELS_PACKED) {
       return;
   }

    uint8_t length = buf[1];
    // Extract payload
    const uint8_t *payload = &buf[3];

    // The CRC is the last byte of the frame
    const uint8_t crcFromPacket = buf[1 + length];
    const uint8_t crcCalc = crsf_crc8(&buf[2], length - 1); // TYPE+PAYLOAD
    
    if (crcFromPacket != crcCalc) {
        return; // corrupted frame, ignore
    }

    // Decode channels
    uint16_t channels[16];
    uint32_t bitBuffer = 0;
    uint8_t bitsInBuffer = 0;
    uint8_t payloadIndex = 0;

    for (int i = 0; i < 16; ++i) {
        while (bitsInBuffer < 11) {
            bitBuffer |= ((uint32_t)payload[payloadIndex++]) << bitsInBuffer;
            bitsInBuffer += 8;
        }
        channels[i] = bitBuffer & 0x7FF; // Extract 11 bits
        bitBuffer >>= 11;
        bitsInBuffer -= 11;

        if (i < 4) { //stick channels
            rcData_.controlSignals[i] = static_cast<float>((channels[i] - CRSF_PULSE_MIN) * (100.0f / CRSF_PULSE_RANGE));
        } 
        else{
            // ARM and AUX channels
            rcData_.controlSignals[i] = static_cast<float>((channels[i] - CRSF_AUX_MIN) * (100.0f / CRSF_AUX_RANGE));
        }
    }

    rcData_.isDataNew = true;
}







RCControl GeminiMavlink::getRCData() {
    RCControl tmp;
    tmp.isDataNew = false;
    return tmp;
}

UART_HandleTypeDef* GeminiMavlink::getHuart() const {
    return huart;
}

void GeminiMavlink::irqhandler(uint16_t size) {
    writeIndex = size;
}

void GeminiMavlink::forcePushMAVLinkRC(RCControl rcData) {
    rcData_ = rcData;
}
