#pragma once

#include "stm32l5xx_hal.h"
#include "airspeed_iface.hpp"
#include <cmath>

enum class Status : uint8_t {
    Normal  = 0b00,
    Command = 0b01,
    Stale   = 0b10,
    Fault   = 0b11
};

struct airspeedData {
    double raw_press_ = 0;
    double raw_temp_ = 0;
    double processed_temp_ = 0;
    double processed_press_ = 0;
    double airspeed_ = 0;
};

class airspeed : public airspeed_iface
{
private:
	static constexpr int arraySize = 4;
    uint8_t dmaRXBuffer[arraySize];
    uint8_t processRXBuffer[arraySize];

    I2C_HandleTypeDef* hi2c;
    uint8_t devAddress;

    double pressZero = 0.0;

    bool callibrate(int samples, int discard);
    bool calibrated_ = false;
    bool initSuccess_ = false;

    Status status_ = Status::Fault;

    airspeedData airspeedData_;

public:
    airspeed(I2C_HandleTypeDef* i2c, uint8_t addr = 0x28) : hi2c(i2c), devAddress(addr << 1) {}
    ~airspeed() = default;

    // init
    bool airspeedInit();

    //helper function to calculate airspeed
    bool calculateAirspeed(double* data_out);
    bool I2C_DMA_CALLBACK();

    // public getters
    bool getAirspeedData(double* data_out);
    I2C_HandleTypeDef* getI2C() { return hi2c; }
    uint8_t* getDMARXBuffer() { return dmaRXBuffer; }
    uint8_t* getProcessRXBuffer() { return processRXBuffer; }
    uint8_t getDevAddress() { return devAddress; }
    uint8_t getArraySize() { return arraySize; }
    airspeedData getAirspeedDataStruct() { return airspeedData_; }
};
