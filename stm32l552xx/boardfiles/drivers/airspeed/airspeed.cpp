#include "airspeed.hpp"
#include <cstring>

/*
airspeedInit - gets the initial data, confirms that data is good
getData - runs only if airspeedInit gives HAL_OK
*/

bool airspeed::airspeedInit() {
    bool success = false;
    bool dma_success = false;
    HAL_StatusTypeDef status = HAL_I2C_Master_Receive_DMA(hi2c, devAddress, dmaRXBuffer, arraySize);
    dma_success = (status == HAL_OK);
    
    if (dma_success) {
        success = callibrate(20, 4);
    }

    initSuccess_ = success;
    return success;
}

bool airspeed::callibrate(int samples, int discard) {
    double sumPress = 0;
    double n = 0;

    for (int i = 0; i < samples + discard; i++) {
        Status status = static_cast<Status>((processRXBuffer[0] >> 6) & 0x03);
        if (status != Status::Normal) continue;
        const double rPress = (((processRXBuffer[0] & 0x3F) << 8) | processRXBuffer[1]);

        //calculate pressure in psi
        //Assumptions: output type A, 30 psi pressure range, differential mode (converted from psi to pa)
        const double pPress = ((rPress - 1638.3)/6553.2 - 1) * 6894.76;

        if (i < discard) continue;

        sumPress += pPress;
        n++;
    }

    if (n < (samples / 2)) return false; // too few good samples

    pressZero = sumPress / n;
    calibrated_ = true;

    return true;
}

bool airspeed::getAirspeedData(double* data_out) {
	if (!initSuccess_) { return false; }
	return calculateAirspeed(data_out); // Will send a proper error message later
}

bool airspeed::calculateAirspeed(double* data_out) {
    status_ = static_cast<Status>((processRXBuffer[0] >> 6) & 0x03);
    if(status_ != Status::Normal) return false; // something wrong with the data

    airspeedData_.raw_press_ = (((processRXBuffer[0] & 0x3F) << 8) | (processRXBuffer[1]));
    airspeedData_.raw_temp_ = ((processRXBuffer[2] << 3) | (((processRXBuffer[3] & 0xE0) >> 5) & 0x03));

    // -- Calculations -- //

    //process raw temperature and pressure data
    airspeedData_.processed_temp_ = (airspeedData_.raw_temp_ * 200)/2047 - 50; // calculate temperature in deg C

    //calculate pressure in psi
    //Assumptions: output type A, 30 psi pressure range, differential mode
    airspeedData_.processed_press_ = (airspeedData_.raw_press_ - 1638.3)/6553.2 - 1;

    //convert pressure to Pa
    if (calibrated_) {
    	airspeedData_.processed_press_ = abs((airspeedData_.processed_press_ * 6894.76) - pressZero);
    }


    double air_density = 101325.0 / (287.058 * (airspeedData_.processed_temp_ + 273.15)); //calculate air density in kg/m^3, assuming stanard air pressure of 101.325 kPa and specific gas constant for dry air R = 287.058 J/(kg·K)

    //calculate airspeed in m/s using Bernoulli's equation
    airspeedData_.airspeed_ = std::sqrt((2 * airspeedData_.processed_press_) / air_density); // this negative symbol might not be right

    *data_out = airspeedData_.airspeed_;
    return true; // Will send a proper error message later
}


bool airspeed::I2C_DMA_CALLBACK() {
    memcpy(
        getProcessRXBuffer(),
        getDMARXBuffer(),
        getArraySize()
	);

    HAL_I2C_Master_Receive_DMA(
        getI2C(),
        getDevAddress(),
        getDMARXBuffer(),
        getArraySize()
    );
}