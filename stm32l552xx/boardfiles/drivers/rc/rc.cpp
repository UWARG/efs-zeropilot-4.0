#include <cmath>
#include <cstring>
#include <cstdio>
#include "rc.hpp"

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

RCReceiver::RCReceiver(UART_HandleTypeDef* uart) : uart(uart) {
    memset(rawSbus, 0, SBUS_PACKET_SIZE);
}


UART_HandleTypeDef* RCReceiver::getHUART() {
    return uart;
}

ZP_ERROR_e RCReceiver::getRCData(RCControl *data) {
    if (data == nullptr) {
        return ZP_ERROR_NULLPTR;
    }

    *data = rcData_;
    rcData_.isDataNew = false;
    return ZP_ERROR_OK;
}

ZP_ERROR_e RCReceiver::init() {
    if (uart_ == nullptr) {
        return ZP_ERROR_NULLPTR;
    }

    // start circular DMA
    rcData_.isDataNew = false;
    HAL_StatusTypeDef hal_status = HAL_UARTEx_ReceiveToIdle_DMA(uart_, rawSbus_, SBUS_BYTE_COUNT);
    if (hal_status != HAL_OK) {
        return ZP_ERROR_FAIL;
    }

    return ZP_ERROR_OK;
}

ZP_ERROR_e RCReceiver::startDMA() {
    if (uart_ == nullptr) {
        return ZP_ERROR_NULLPTR;
    }

    // start circular DMA
    HAL_StatusTypeDef hal_status = HAL_UARTEx_ReceiveToIdle_DMA(uart_, rawSbus_, SBUS_BYTE_COUNT);
    if (hal_status != HAL_OK) {
        return ZP_ERROR_FAIL;
    }

    return ZP_ERROR_OK;
}

ZP_ERROR_e RCReceiver::parse() {
    uint8_t *buf = rawSbus_;

    if ((buf[0] == HEADER_) && (buf[24] == FOOTER_)) {
        for (int i = 0; i < SBUS_CHANNEL_COUNT; i++) {
            ZP_ERROR_e err = sbusToRCControl(&rcData_.controlSignals[i], buf, i);
            if (err != ZP_ERROR_OK) {
                return err;
            }
        }

        rcData_.isDataNew = true;
    } else {
        return ZP_ERROR_PARSE;
    }

    return ZP_ERROR_OK;
}

ZP_ERROR_e RCReceiver::sbusToRCControl(float *value, uint8_t *buf, int channelMappingIdx) {
    if (value == nullptr || buf == nullptr) {
        return ZP_ERROR_NULLPTR;
    }

    if (channelMappingIdx < 0 || channelMappingIdx >= SBUS_CHANNEL_COUNT) {
        return ZP_ERROR_INVALID_PARAM;
    }

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

    *value = static_cast<float>((res - SBUS_RANGE_MIN) * (100.0f / SBUS_RANGE_RANGE));
    return ZP_ERROR_OK;
}
