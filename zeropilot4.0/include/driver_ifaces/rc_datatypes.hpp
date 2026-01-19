#pragma once

#include <algorithm>

#define INPUT_CHANNELS 16

/*  control signal channel mapping and attribute values */
/*  all values are floats and range from 0 to 100 */
class RCControl {
    public:

        float controlSignals[INPUT_CHANNELS];
        bool isDataNew;

        float &roll     = controlSignals[0];
        float &pitch    = controlSignals[1];
        float &throttle = controlSignals[2];
        float &yaw      = controlSignals[3];
        float &arm      = controlSignals[4];
        float &aux1     = controlSignals[5];
        float &aux2     = controlSignals[6];
        float &aux3     = controlSignals[7];
        float &aux4     = controlSignals[8];
        float &aux5     = controlSignals[9];
        float &aux6     = controlSignals[10];
        float &aux7     = controlSignals[11];
        float &aux8     = controlSignals[12];
        float &aux9     = controlSignals[13];
        float &aux10    = controlSignals[14];
        float &aux11    = controlSignals[15];

        RCControl operator=(const RCControl& other){
            std::copy(other.controlSignals, other.controlSignals + INPUT_CHANNELS, this->controlSignals);
            this->isDataNew = other.isDataNew; // Add this line to copy the isDataNew flag
            return *this;
        }

        RCControl() {
            isDataNew = false;
            roll      = 50.0f;
            pitch     = 50.0f;
            throttle  = 0.0f;
            yaw       = 50.0f;
            arm       = 0.0f;
            aux1      = 0.0f;
            aux2      = 0.0f;
            aux3      = 0.0f;
            aux4      = 0.0f;
            aux5      = 0.0f;
            aux6      = 0.0f;
            aux7      = 0.0f;
            aux8      = 0.0f;
            aux9      = 0.0f;
            aux10     = 0.0f;
            aux11     = 0.0f;
        }
};
