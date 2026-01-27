#include <cmath>
#include <cstring>
#include <cstdio>
#include "rc_defines.hpp"
#include "rc_crsf.hpp"

CRSFReceiver::CRSFReceiver(UART_HandleTypeDef* uart) : uart_(uart) {
    memset(crsfRxBuffer_, 0, CRSF_PACKET_SIZE);
}

RCControl CRSFReceiver::getRCData() {
    RCControl tmp = rcData_;
    rcData_.isDataNew = false;
    return tmp;
}

void CRSFReceiver::init() {
    // start circular DMA
    rcData_.isDataNew = false;
    HAL_UARTEx_ReceiveToIdle_DMA(uart_, crsfRxBuffer_, CRSF_PACKET_SIZE);
}

void CRSFReceiver::startDMA() {
    // start circular DMA
    HAL_UARTEx_ReceiveToIdle_DMA(uart_, crsfRxBuffer_, CRSF_PACKET_SIZE);
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

void CRSFReceiver::parse() {

    uint8_t *buf = crsfRxBuffer_;

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

        // Map to desired range
        rcData_.controlSignals[i] = static_cast<float>((channels[i] - CRSF_PULSE_MIN) * (100.0f / CRSF_PULSE_RANGE));
    }

    rcData_.isDataNew = true;
}

UART_HandleTypeDef * CRSFReceiver::getHUART() {
    return uart_;
}