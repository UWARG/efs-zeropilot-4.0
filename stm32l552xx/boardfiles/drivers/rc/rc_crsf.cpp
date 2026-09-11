#include <cmath>
#include <cstring>
#include <cstdio>
#include "rc_defines.hpp"
#include "rc_crsf.hpp"

CRSFReceiver::CRSFReceiver(UART_HandleTypeDef* uart) : uart(uart) {
    memset(crsfRxBuffer, 0, CRSF_PACKET_SIZE);
}

ZP_Error CRSFReceiver::getRCData(RCControl &data) {
    RCControl tmp = rcData;
    rcData.isDataNew = false;
    data = tmp;
    return ZP_ERROR_OK;
}

ZP_Error CRSFReceiver::init() {
    rcData.isDataNew = false;
    return startDMA();
}

ZP_Error CRSFReceiver::startDMA() {
    if (uart == nullptr) {
        return ZP_ERROR_NULLPTR;
    }

    // start circular DMA
    HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(uart, crsfRxBuffer, CRSF_PACKET_SIZE);
    if (status == HAL_BUSY) {
        return ZP_ERROR_EXT_API | ZP_ERROR_BUSY;
    } else if (status != HAL_OK) {
        return ZP_ERROR_EXT_API | ZP_ERROR_FAIL;
    }
    return ZP_ERROR_OK;
}

// Polynomial used in CRSF: 0xD5
static ZP_Error crsf_crc8(const uint8_t *data, uint8_t len, uint8_t &output) {
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
    output = crc;
    return ZP_ERROR_OK;
}

ZP_Error CRSFReceiver::parse() {
    ZP_Error result = ZP_ERROR_OK;
    uint8_t *buf = crsfRxBuffer;

    // Validate sync byte and frame type
   if (buf[0] != CRSF_SYNC_BYTE || buf[2] != CRSF_FRAMETYPE_RC_CHANNELS_PACKED) {
       return ZP_ERROR_PARSE;
   }

    uint8_t length = buf[1];
    // Extract payload
    const uint8_t *payload = &buf[3];

    // The CRC is the last byte of the frame
    const uint8_t crcFromPacket = buf[1 + length];
    uint8_t crcCalc = 0;
    result |= crsf_crc8(&buf[2], length - 1, crcCalc); // TYPE+PAYLOAD
    
    if (crcFromPacket != crcCalc || result != ZP_ERROR_OK) {
        return result; // corrupted frame, ignore
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
            rcData.controlSignals[i] = static_cast<float>((channels[i] - CRSF_PULSE_MIN) * (100.0f / CRSF_PULSE_RANGE));
        } 
        else {
            // ARM and AUX channels
            rcData.controlSignals[i] = static_cast<float>((channels[i] - CRSF_AUX_MIN) * (100.0f / CRSF_AUX_RANGE));
        }
    }

    rcData.isDataNew = true;
    return result;
}

UART_HandleTypeDef * CRSFReceiver::getHuart() {
	return uart;
}
