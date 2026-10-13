#pragma once

#include "magnetometer_iface.hpp"
#include "ist8310_i2c.hpp"
#include "stm32l5xx_hal.h"

class Ist8310Magnetometer : public IMagnetometer {
public:
    explicit Ist8310Magnetometer(Ist8310 *driver) : driver_(driver) {}

    bool init() override {
        return driver_->init();
    }

    MagData_t readData() override {
        MagData_t data = {};
        Ist8310::Sample sample = {};
        data.isNew = driver_->read(sample);
        if (data.isNew) {
            data.x = sample.x_microtesla;
            data.y = sample.y_microtesla;
            data.z = sample.z_microtesla;
            data.rawX = sample.x_raw;
            data.rawY = sample.y_raw;
            data.rawZ = sample.z_raw;
            data.timestamp = HAL_GetTick();
        }
        return data;
    }

private:
    Ist8310 *driver_;
};
