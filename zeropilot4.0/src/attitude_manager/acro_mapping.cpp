#include <cmath>
#include "acro_mapping.hpp"
#include "unit_conversions.hpp"

ACROMapping::ACROMapping(float control_iter_period_s) noexcept : rollPID(0.0f, 0.0f, 0.0f, 0.0f,
                                                                         OUTPUT_MIN, OUTPUT_MAX, 100,
                                                                         control_iter_period_s),
                                                                 pitchPID(0.0f, 0.0f, 0.0f, 0.0f,
                                                                          OUTPUT_MIN, OUTPUT_MAX, 100,
                                                                          control_iter_period_s),
                                                                 yawPID(0.0f, 0.0f, 0.0f, 0.0f,
                                                                        OUTPUT_MIN, OUTPUT_MAX, 100,
                                                                        control_iter_period_s),
                                                                 rollLimitRate(0.0f),
                                                                 pitchLimitRate(0.0f),
                                                                 yawLimitRate(0.0f)
{
    rollPID.pidInitState();
    pitchPID.pidInitState();
    yawPID.pidInitState();
}

// Setter *roll* for PID consts
void ACROMapping::setRollPIDConstants(float newKp, float newKi, float newKd, float newTau, uint8_t newIMaxPct) noexcept
{
    rollPID.setConstants(newKp, newKi, newKd, newTau, newIMaxPct);
}

// Setter for *pitch* PID consts
void ACROMapping::setPitchPIDConstants(float newKp, float newKi, float newKd, float newTau, uint8_t newIMaxPct) noexcept
{
    pitchPID.setConstants(newKp, newKi, newKd, newTau, newIMaxPct);
}

// Setter for *yaw* PID consts
void ACROMapping::setYawPIDConstants(float newKp, float newKi, float newKd, float newTau, uint8_t newIMaxPct) noexcept
{
    yawPID.setConstants(newKp, newKi, newKd, newTau, newIMaxPct);
}

// Resetter for both roll and pitch PIDs (needed for unit testing)
void ACROMapping::resetControlLoopState() noexcept
{
    rollPID.pidInitState();
    pitchPID.pidInitState();
    yawPID.pidInitState();
}

// Setter for *rollLimitRate* in deg / s
void ACROMapping::setRollLimitRate(float newRollLimitRate) noexcept
{
    rollLimitRate = newRollLimitRate;
}

// Setter for *pitchLimitRate* in deg / s
void ACROMapping::setPitchLimitRate(float newPitchLimitRate) noexcept
{
    pitchLimitRate = newPitchLimitRate;
}

// Setter for *rollLimitRate* in deg / s
void ACROMapping::setYawLimitRate(float newYawLimitRate) noexcept
{
    yawLimitRate = newYawLimitRate;
}

// Getter for PID objects
PID *ACROMapping::getRollPID() noexcept { return &rollPID; }
PID *ACROMapping::getPitchPID() noexcept { return &pitchPID; }
PID *ACROMapping::getYawPID() noexcept { return &yawPID; }

void ACROMapping::activateFlightMode()
{
    resetControlLoopState();
}

// Main control mapping function for ACRO mode
RCMotorControlMessage_t ACROMapping::runControl(RCMotorControlMessage_t controlInputs, const DroneState_t &droneState)
{
    // Setpoints: Maps [0, 100] to [-limit, +limit]
    float rollRateSetpoint = ((controlInputs.roll / MAX_RC_INPUT_VAL) * 2.0f - 1.0f) * rollLimitRate;
    float pitchRateSetpoint = ((controlInputs.pitch / MAX_RC_INPUT_VAL) * 2.0f - 1.0f) * pitchLimitRate;
    float yawRateSetpoint = ((controlInputs.yaw / MAX_RC_INPUT_VAL) * 2.0f - 1.0f) * yawLimitRate;

    float rollRateMeasured = droneState.rollRate;
    float pitchRateMeasured = droneState.pitchRate;
    float yawRateMeasured = droneState.yawRate;

    // // Run PID, outputs control effort in [-1,1]
    controlInputs.roll = rollPID.pidOutput(rollRateSetpoint, rollRateMeasured);
    controlInputs.pitch = pitchPID.pidOutput(pitchRateSetpoint, pitchRateMeasured);
    controlInputs.yaw = yawPID.pidOutput(yawRateSetpoint, yawRateMeasured);
    controlInputs.throttle /= 100.0f; 

    // Run motor mixing
    motorMixer(controlInputs);

    return controlInputs;
}

void ACROMapping::motorMixer(const RCMotorControlMessage_t outputControlMsg)
{
    // roll, pitch, yaw in range [-1, 1], throttle in [0,1]
    float roll = outputControlMsg.roll;
    float pitch = outputControlMsg.pitch;
    float yaw = outputControlMsg.yaw;
    float throttle = outputControlMsg.throttle; 

    static const int8_t roll_sign[4] = { -1, 1, 1, -1};
    static const int8_t pitch_sign[4] = { 1, -1, 1, -1};
    static const int8_t yaw_sign[4] = { 1, 1, -1, -1};

    const float YAW_HEADROOM = 0.02f;

    float max = 0.0f;
    float min = 0.0f;
    // Roll and Pitch
    for (int i = 0; i < 4; i++) {
        motor_percent[i] = roll * roll_sign[i] + pitch * pitch_sign[i];
        max = fmaxf(max, motor_percent[i]);
        min = fminf(min, motor_percent[i]);
    }
    // reduce roll and pitch if leaving no room for yaw
    float range = max - min;
    if (range > 1.0f - YAW_HEADROOM) {
        float scaling_factor = (1.0f - YAW_HEADROOM) / range;
        for (int i = 0; i < 4; i++) {
            motor_percent[i] *= scaling_factor;
        }
    }

    min = 0.0f;
    max = 0.0f;
    // Yaw
    for(int i = 0; i < 4; i++) {
        motor_percent[i] += yaw * yaw_sign[i];
        max = fmaxf(max, motor_percent[i]);
        min = fminf(min, motor_percent[i]);
    }
    range = max - min;
    // reduce roll, pitch and yaw together if adding yaw saturates
    if (range > 1.0f ) {
        float scaling_factor = 1.0f / range;
        min = 0.0f;
        max = 0.0f;
        for (int i = 0; i < 4; i++) {
            motor_percent[i] *= scaling_factor;
            max = fmaxf(max, motor_percent[i]);
            min = fminf(min, motor_percent[i]);     
        }
    }

    // Throttle
    float min_throttle = fmaxf(-min, throttle);     // Add enough throttle to keep all motors >= 0
    float max_throttle = 1.0f - max;
    throttle = fminf(min_throttle, max_throttle);
    for(int i = 0; i < 4; i++) {
        motor_percent[i] += throttle;
    }


//     motor_percent[0] = -roll + pitch;
//     motor_percent[1] = roll - pitch;
//     motor_percent[2] = roll + pitch;
//     motor_percent[3] = -roll - pitch;

//     bool disregard_yaw_flag = false;

//     // Roll and Pitch
//     float max_range = 0.0f;
//     // max range
//     for (int i = 0; i < 4; i++) {
//         if (fabsf(motor_percent[i]) > max_range) {
//             max_range = fabsf(motor_percent[i]);
//         }
//     }
//     // scale down to [-0.5,0.5]
//     if (max_range > 0.5f) {
//         float scaling_factor = 0.5 / max_range;
//         for (int i = 0; i < 4; i++) {
//             motor_percent[i] *= scaling_factor;
//         }
//     }

//     // Throttle
//     float min_throttle = 0.0f;
//     for (int i = 0; i < 4; i++) {
//         if (motor_percent[i] < 0 && fabsf(motor_percent[i]) > min_throttle) {
//             min_throttle = fabsf(motor_percent[i]);
//         }
//     }
//     if (throttle < min_throttle) { throttle = min_throttle; }

//     float max_overshoot = 0.0f;
//     for (int i = 0; i < 4; i++) {
//         float overshoot = motor_percent[i] + throttle - 1;
//         if (overshoot > max_overshoot) {
//             max_overshoot = overshoot;
//         }
//     }
//     if (max_overshoot > 0) {
//         // Decrease throttle to fit in [0,1]
//         for (int i = 0; i < 4; i++) {
//             motor_percent[i] += throttle - max_overshoot;
//         }
//         disregard_yaw_flag = true;
//     } else {
//         // Throttle does not cause saturation
//         for (int i = 0; i < 4; i++) {
//             motor_percent[i] += throttle;
//         }
//     }

//     // Yaw
//     if (!disregard_yaw_flag) {
//         float min_yaw_scale = 1.0f;
//         int8_t yaw_signs[4] = { 1, 1, -1, -1 };
//         for(int i = 0; i < 4; i++) {
//             float yaw_contribution = yaw * yaw_signs[i]; // yaw contribution could be neg or pos
//             if (yaw_contribution > 0) {
//                 // Saturates above 0 
//                 float yaw_scale = (1.0f - motor_percent[i]) / yaw_contribution;  
//                 if( yaw_scale < min_yaw_scale) {
//                     min_yaw_scale = yaw_scale;
//                 }
//             } else if (yaw_contribution < 0){
//                 // Saturates below 0
//                 float yaw_scale = motor_percent[i] / (-yaw_contribution);  
//                 if( yaw_scale < min_yaw_scale) {
//                     min_yaw_scale = yaw_scale;
//                 }
//             }
//         }

//         for (int i = 0; i < 4; i++) {
//             motor_percent[i] += yaw_signs[i] * yaw * min_yaw_scale;
//         }
//     }

}

const float *ACROMapping::getMixedMotors() {
    return motor_percent;
}