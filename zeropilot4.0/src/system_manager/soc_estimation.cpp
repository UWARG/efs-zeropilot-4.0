#include "system_manager.hpp"
#include "zp_params.hpp"

SocEstimator::SocEstimator(BatteryData_t batteryData){
    calcStateOfCharge(batteryData, SOC_IDLE_MODE);
}

uint8_t SocEstimator::getSocPercentage(){
    return socData.socPercentage;
}

int32_t SocEstimator::getTimeRemaining(){
    return socData.timeRemaining;
}

void SocEstimator::calcStateOfCharge(BatteryData_t batteryData, int mode) {
    float currVoltage = batteryData.pmData.busVoltage;            
    float batteryCharge = ZP_PARAM::get(ZP_PARAM_ID::BATT_CAPACITY) * 3.6f; // mA to C
    float remainingCharge = batteryCharge - batteryData.pmData.charge;

    // Calibrate SOC
    if (currVoltage <= V_MIN*(ZP_PARAM::get(ZP_PARAM_ID::BATT_N_CELLS))) socData.socPercentage = 0;
    else if (currVoltage >= V_MAX*(ZP_PARAM::get(ZP_PARAM_ID::BATT_N_CELLS))) socData.socPercentage = 100;
    
    // Calculate SOC
    else {
        // State 1: Interpolate
        if (mode == SOC_IDLE_MODE){
            // linear search to find points to linearly interpolate
            size_t i = 0;
            while (i < SOC_LUT_SIZE && SOC_LUT[i].voltage*(ZP_PARAM::get(ZP_PARAM_ID::BATT_N_CELLS)) < currVoltage) i += 1;
            
            // linear interpolation
            if (i >= SOC_LUT_SIZE) socData.socPercentage = 100; // Assume 100% SOC
            else if (i == 0) socData.socPercentage = 0; // Assume 0% SOC
            else{
                const auto& pointA = SOC_LUT[i-1];
                const auto& pointB = SOC_LUT[i];
                float soc  = (pointB.soc - pointA.soc)/(pointB.voltage-pointA.voltage)*(currVoltage-pointA.voltage)+pointA.soc;
                soc = soc > 100.0f ? 100.0f : soc < 0.0f ? 0.0f : soc;
                socData.socPercentage = static_cast<uint8_t>(soc);
            }
        }

        // State 2: Charge Cycle
        if (mode == SOC_CHARGE_DISCHARGE_MODE) {    
            // Charge Method - Non-iterative, Uses accumulated board charge
            float socPercentage = (remainingCharge / batteryCharge) * 100.0f;
            socData.socPercentage = static_cast<uint8_t>((socPercentage > 100.0f) ? 100.0f : (socPercentage < 0.0f) ? 0.0f : socPercentage);
        } 
    }

    // Calculate Time Remaining
    if (batteryData.pmData.current > 0.5f) {
        socData.timeRemaining = static_cast<int32_t>(remainingCharge / batteryData.pmData.current);
    } else {
        socData.timeRemaining = 0;
    }
}