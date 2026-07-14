#include "soc_estimation.hpp"
#include "system_manager.hpp"
#include "zp_params.hpp"

SocEstimator::SocEstimator(BatteryData_t batteryData) {}

uint8_t SocEstimator::getSocPercentage(){
    return socData.socPercentage;
}

int32_t SocEstimator::getTimeRemaining(){
    return socData.timeRemaining;
}

void SocEstimator::calcStateOfCharge(BatteryData_t batteryData, int mode) {
    float currVoltage = batteryData.pmData.busVoltage;            
    float batteryCharge = ZP_PARAM::get(ZP_PARAM_ID::BATT_CAPACITY) * 3.6f; // mA to C
    uint8_t nCells = ZP_PARAM::get(ZP_PARAM_ID::BATT_N_CELLS);
    
    if (nCells == 0) return; // Prevent division by zero
    
    // Force idle calibration on the first valid telemetry tick
    if (initialSocPercentage < 0.0f && currVoltage > 1.0f) {
        mode = SOC_IDLE_MODE;
    }

    // Convert to per-cell voltage for the LUT
    float cellVoltage = currVoltage / nCells;

    // Calibrate SOC bounds
    if (cellVoltage <= V_MIN) socData.socPercentage = 0;
    else if (cellVoltage >= V_MAX) socData.socPercentage = 100;
    
    // Calculate SOC
    else {
        // State 1: Voltage Interpolation
        if (mode == SOC_IDLE_MODE){
            size_t i = 0;
            // Fixed loop condition: look for first voltage smaller than current
            while (i < SOC_LUT_SIZE && SOC_LUT[i].voltage > cellVoltage) i += 1;
            
            if (i >= SOC_LUT_SIZE) socData.socPercentage = 0; 
            else if (i == 0) socData.socPercentage = 100; 
            else{
                const auto& pointA = SOC_LUT[i-1];
                const auto& pointB = SOC_LUT[i];

                // Linear interpolation between two points
                float soc  = (pointB.soc - pointA.soc)/(pointB.voltage-pointA.voltage)*(cellVoltage-pointA.voltage)+pointA.soc;
                soc = soc > 100.0f ? 100.0f : soc < 0.0f ? 0.0f : soc;
                socData.socPercentage = static_cast<uint8_t>(soc);
            }
            initialSocPercentage = socData.socPercentage;
        }

        // State 2: Charge Cycle (Coulomb Counting)
        if (mode == SOC_CHARGE_DISCHARGE_MODE && initialSocPercentage >= 0.0f) {    
            // Fixed starting point: use the voltage-calibrated SOC, not 100%
            float startingCharge = (initialSocPercentage / 100.0f) * batteryCharge;
            float remainingCharge = startingCharge - batteryData.pmData.charge;
            float socPercentage = (remainingCharge / batteryCharge) * 100.0f;
            socData.socPercentage = static_cast<uint8_t>((socPercentage > 100.0f) ? 100.0f : (socPercentage < 0.0f) ? 0.0f : socPercentage);
        } 
    }

    // Calculate Time Remaining based on calibrated capacity
    float startingCharge = (initialSocPercentage >= 0.0f) ? ((initialSocPercentage / 100.0f) * batteryCharge) : batteryCharge;
    float remainingCharge = startingCharge - batteryData.pmData.charge;
    
    if (batteryData.pmData.current > 0.5f) {
        socData.timeRemaining = static_cast<int32_t>(remainingCharge / batteryData.pmData.current);
    } else {
        socData.timeRemaining = 0;
    }
}
