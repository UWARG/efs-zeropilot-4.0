#pragma once

#include "stm32h7xx_hal.h"
#include "airspeed_iface.hpp"
#include <cmath>

class Airspeed : public IAirspeed {
private:
	static constexpr int arraySize = 4;
    uint8_t dmaRXBuffer[arraySize];
    uint8_t processRXBuffer[arraySize];

    I2C_HandleTypeDef* hi2c;
    uint8_t devAddress;

    double pressZero = 0.0;

    bool calibrate(int samples, int discard);
    bool calibrated_ = false;
    bool initSuccess_ = false;

    Status status_ = Status::Fault;

public:
    Airspeed(I2C_HandleTypeDef* i2c, uint8_t addr = 0x28) : hi2c(i2c), devAddress(addr << 1) {}
    ~Airspeed() = default;

    bool init();

    //helper function to calculate airspeed
    bool calculateAirspeed(AirspeedData_t* airspeedData_);
    void receiveCallback();

    // public getters
    bool getAirspeedData(AirspeedData_t* data) override;
    
    I2C_HandleTypeDef* getHI2C() { return hi2c; }
    uint8_t* getDMARXBuffer() { return dmaRXBuffer; }
    uint8_t* getProcessRXBuffer() { return processRXBuffer; }
    uint8_t getDevAddress() { return devAddress; }
    uint8_t getArraySize() { return arraySize; }
//    AirspeedData_t getAirspeedDataStruct() { return airspeedData_; }
};
