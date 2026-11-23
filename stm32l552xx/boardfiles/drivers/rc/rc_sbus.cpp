#include <cmath>
#include <cstring>
#include <cstdio>
#include "rc_sbus.hpp"

DataChunk_t channelMappings[SBUS_CHANNEL_COUNT][SBUS_MAX_BTYES_PER_CHANNEL] = {
    { //channel 1
        {1, 0xFF, 0}, {2, 0x07, 8}, {0, 0, 0}
    },
    { //channel 2
        {2, 0xF8, -3}, {3, 0x3F, 5}, {0, 0, 0}
    },
    { //channel 3
        {3, 0xC0, -6}, {4, 0xFF, 2}, {5, 0x01, 10}
    },
    { //channel 4
        {5, 0xFE, -1}, {6, 0x0F, 7}, {0, 0 , 0}
    },
    { //channel 5
        {6, 0xF0, -4}, {7, 0x7F, 4}, {0, 0, 0}
    },
    { //channel 6
        {7, 0x80, -7}, {8, 0xFF, 1}, {9, 0x03, 9}
    },
    { //channel 7
        {9, 0xFC, -2}, {10, 0x1F, 6}, {0, 0, 0}
    },
    { //channel 8
        {10, 0xe0, -5}, {11, 0xFF, 3}, {0, 0, 0}
    },
    { //channel 9
        {12, 0xFF, 0}, {13, 0x07, 8}, {0, 0, 0}
    },
    { //channel 10
        {13, 0xF8, -3}, {14, 0x3F, 5}, {0, 0, 0}
    },
    { //channel 11
        {14, 0xC0, -6}, {15, 0xFF, 2}, {16, 0x01, 10}
    },
    { //channel 12
        {16, 0xFE, -1}, {17, 0x0F, 7}, {0, 0, 0}
    },
    { //channel 13
        {17, 0xF0, -4}, {18, 0x7F, 4}, {0, 0, 0}
    },
    { //channel 14
        {18, 0x80, -7}, {19, 0xFF, 1}, {20, 0x03, 9}
    },
    { //channel 15
        {20, 0xFC, -2}, {21, 0x1F, 6}, {0, 0, 0}
    },
    { //channel 16
        {21, 0xe0, -5}, {22, 0xFF, 3}, {0, 0, 0}
    }
};

SBUSReceiver::SBUSReceiver(UART_HandleTypeDef* uart) : uart(uart) {
    memset((void*)rawSbus, 0, SBUS_PACKET_SIZE);
}


UART_HandleTypeDef* SBUSReceiver::getHuart() {
    return uart;
}

ZP_Error SBUSReceiver::getRCData(RCControl &data) {
    data = rcData;
    rcData.isDataNew = false;
    return ZP_ERROR_OK;
}

ZP_Error SBUSReceiver::init() {
    rcData.isDataNew = false;
    return startDMA();
}

ZP_Error SBUSReceiver::startDMA() {
    if (uart == nullptr) {
        return ZP_ERROR_NULLPTR;
    }

    HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(uart, (uint8_t*)rawSbus, SBUS_PACKET_SIZE);
    if (status == HAL_BUSY) {
        return ZP_ERROR_EXT_API | ZP_ERROR_BUSY;
    } else if (status != HAL_OK) {
        return ZP_ERROR_EXT_API | ZP_ERROR_FAIL;
    }
    return ZP_ERROR_OK;
}

ZP_Error SBUSReceiver::parse() {
    ZP_Error result = ZP_ERROR_OK;
    uint8_t *buf = (uint8_t*)rawSbus;
    float sbusResult = 0.0f;

    if ((buf[0] == HEADER_) && (buf[SBUS_PACKET_SIZE-1] == FOOTER_)) {

        for (int i = 0; i < SBUS_CHANNEL_COUNT; i++) {
            result |= sbusToRCControl(buf, i, sbusResult);
            if (result == ZP_ERROR_OK) {
                rcData.controlSignals[i] = sbusResult;
            } else {
                break;
            }
        }
        if (result == ZP_ERROR_OK) {    
            rcData.isDataNew = true;
        }
    } else {
        result |= ZP_ERROR_PARSE;
    }

    return result;
}

ZP_Error SBUSReceiver::sbusToRCControl(uint8_t *buf, int channelMappingIdx, float &output) {
    ZP_Error result = ZP_ERROR_OK;
    uint16_t res = 0;

    for (int i = 0; i < SBUS_MAX_BTYES_PER_CHANNEL; i++) {
        DataChunk_t d = channelMappings[channelMappingIdx][i];
        
        uint16_t tmp = d.bitshift >= 0 ?
            (buf[d.dataOffset] & d.mask) << d.bitshift :
            (buf[d.dataOffset] & d.mask) >> abs(d.bitshift);
        
        res |= tmp;
    }

    if(res < SBUS_RANGE_MIN) {
        res = SBUS_RANGE_MIN;
    }
    else if(res > SBUS_RANGE_MAX) {
        res = SBUS_RANGE_MAX;
    }
    else {
        // continue
    }

    output = static_cast<float>((res - SBUS_RANGE_MIN) * (100.0f / SBUS_RANGE_RANGE));
    return result;
}
