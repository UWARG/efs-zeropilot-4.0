#pragma once

#include "rangefinder_iface.hpp"
#include "sitl_driver_configs.hpp"

#include <cstdint>

class SITL_Rangefinder : public IRangefinder {
private:
    using Config = SITL_Driver_Configs::SITL_Rangefinder_Config;

    RangefinderData_t data = {};
    uint32_t usSinceFrame = 0;

public:

    int init() override { return 0; }

    /*
    Mirrors tf02pro.cpp: isNew is only true on the single call that consumes a fresh frame, so the flight
    code sees the sensor's real sample rate instead of the control loop rate. Without this the althold
    surface tracking would never time out and its reacquire debounce would finish 10x too fast.
    */
    RangefinderData_t readData() override {
        RangefinderData_t sample = data;
        data.isNew = false; // Sample handed out, don't report it as new again until the next frame
        return sample;
    }

    void update_from_plant(float sim_altitude, uint32_t dt_us) {
        // Only produce a frame at the sensor's rate, timed off the plant clock so the rate holds
        // regardless of how fast the SITL script manages to step
        usSinceFrame += dt_us;
        if (usSinceFrame < Config::FRAME_PERIOD_US) {
            return;
        }
        usSinceFrame = 0;

        data.distance = sim_altitude;
        data.signalStrength = 65535;
        data.isValid = sim_altitude < 40.0f ? true : false;
        data.isNew = true;
    }
};
