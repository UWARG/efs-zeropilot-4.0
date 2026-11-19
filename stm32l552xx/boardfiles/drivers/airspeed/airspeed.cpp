#include "airspeed.hpp"
#include <cstring>

bool Airspeed::init() {
    if (hi2c == nullptr) return false;

    calibrated_ = false;
    dataValid_ = false;
    filterInitialized_ = false;
    calibrationPressureSum = 0.0;
    calibrationSampleCount = 0;
    pressZero = 0.0;
    lastSampleTick_ = 0;
    errorCount_ = 0;

    if (HAL_I2C_IsDeviceReady(hi2c, devAddress, 3, 100) != HAL_OK) {
        initSuccess_ = false;
        return false;
    }

    initSuccess_ = startReceive();
    return initSuccess_;
}

bool Airspeed::getAirspeedData(AirspeedData_t* data) {
    if (data == nullptr) return false;

    // if a read had previously failed, reinit it so reading doesn't continuously return false
    if (!initSuccess_) {
        initSuccess_ = startReceive();
    }

    // The DMA callback runs in interrupt context and writes 64-bit fields.
    // Briefly mask interrupts so the task receives one coherent snapshot.
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    const bool valid =
        initSuccess_ && calibrated_ && dataValid_ &&
        ((HAL_GetTick() - lastSampleTick_) <= dataTimeoutMs);
    if (valid) *data = latestData_;

    if (primask == 0U) __enable_irq();
    return valid;
}

bool Airspeed::calculateAirspeed(AirspeedData_t* airspeedData_) {
    return getAirspeedData(airspeedData_);
}

bool Airspeed::startReceive() {
    return HAL_I2C_Master_Receive_DMA(
        hi2c,
        devAddress,
        dmaRXBuffer,
        arraySize
    ) == HAL_OK;
}

bool Airspeed::decodeFrame(const uint8_t* frame, AirspeedData_t* data) {
    status_ = static_cast<AsStatus_e>((frame[0] >> 6) & 0x03);
    if (status_ != AsStatus_e::NORMAL) return false;

    data->rawPress = ((frame[0] & 0x3F) << 8) | frame[1];
    data->rawTemp = (frame[2] << 3) | (frame[3] >> 5);
    data->processedTemp = (data->rawTemp * 200.0 / 2047.0) - 50.0;

    // TE 4525DO-DS5AI001DP: type A output, bidirectional +/-1 psi.
    data->processedPress =
        (((data->rawPress - 1638.3) / 6553.2) - 1.0) * 6894.757;
    return true;
}

void Airspeed::receiveCallback() {
    std::memcpy(processRXBuffer, dmaRXBuffer, arraySize);

    AirspeedData_t sample{};
    if (decodeFrame(processRXBuffer, &sample)) {
        if (!calibrated_) {
            calibrationPressureSum += sample.processedPress;
            calibrationSampleCount++;

            if (calibrationSampleCount >= calibrationSamples) {
                pressZero = calibrationPressureSum / calibrationSampleCount;
                calibrated_ = true;
            }
        } else {
            const double correctedPressure = sample.processedPress - pressZero;

            if (!filterInitialized_) {
                filteredPressure = correctedPressure;
                filterInitialized_ = true;
            } else {
                filteredPressure += filterAlpha * (correctedPressure - filteredPressure);
            }

            const double dynamicPressure = filteredPressure > pressureDeadbandPa
                ? filteredPressure - pressureDeadbandPa
                : 0.0;

            sample.processedPress = filteredPressure;
            sample.airspeed = std::sqrt((2.0 * dynamicPressure) / airDensityKgM3);

            latestData_ = sample;
            lastSampleTick_ = HAL_GetTick();
            dataValid_ = true;
        }
    }

    if (!startReceive()) {
        errorCount_++;
        initSuccess_ = false;
    }
}

void Airspeed::errorCallback() {
    errorCount_++;
    initSuccess_ = startReceive();
}
