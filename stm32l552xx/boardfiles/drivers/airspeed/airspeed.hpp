#pragma once

#include "stm32l5xx_hal.h"
#include "airspeed_iface.hpp"
#include <cmath>

class Airspeed : public IAirspeed {
private:
    static constexpr uint8_t arraySize = 4;
    static constexpr uint16_t calibrationSamples = 50;
    static constexpr double pressureDeadbandPa = 5.0;
    static constexpr double filterAlpha = 0.20;
    static constexpr double airDensityKgM3 = 1.225;
    static constexpr uint32_t dataTimeoutMs = 250;

    uint8_t dmaRXBuffer[arraySize]{};
    uint8_t processRXBuffer[arraySize]{};

    I2C_HandleTypeDef* hi2c;
    uint8_t devAddress;

    double pressZero = 0.0;
    double calibrationPressureSum = 0.0;
    double filteredPressure = 0.0;
    uint16_t calibrationSampleCount = 0;
    AirspeedData_t latestData_{};

    bool calibrated_ = false;
    bool initSuccess_ = false;
    bool dataValid_ = false;
    bool filterInitialized_ = false;
    volatile uint32_t lastSampleTick_ = 0;
    volatile uint32_t errorCount_ = 0;

    Status status_ = Status::Fault;

    bool startReceive();
    bool decodeFrame(const uint8_t* frame, AirspeedData_t* data);

public:
    Airspeed(I2C_HandleTypeDef* i2c, uint8_t addr = 0x28) : hi2c(i2c), devAddress(addr << 1) {}
    ~Airspeed() = default;

    bool init();

    // Copies the latest calibrated sample into the caller's buffer.
    bool calculateAirspeed(AirspeedData_t* airspeedData_);
    void receiveCallback();
    void errorCallback();

    // public getters
    bool getAirspeedData(AirspeedData_t* data) override;
    
    I2C_HandleTypeDef* getHI2C() { return hi2c; }
    uint8_t* getDMARXBuffer() { return dmaRXBuffer; }
    uint8_t* getProcessRXBuffer() { return processRXBuffer; }
    uint8_t getDevAddress() { return devAddress; }
    uint8_t getArraySize() { return arraySize; }
    uint32_t getErrorCount() const { return errorCount_; }
//    AirspeedData_t getAirspeedDataStruct() { return airspeedData_; }
};
