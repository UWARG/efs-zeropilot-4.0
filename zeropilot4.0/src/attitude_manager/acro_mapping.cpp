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

// Setter for *rollLimitRate* in rad / s
void ACROMapping::setRollLimitRate(float newRollLimitRate) noexcept
{
    rollLimitRate = newRollLimitRate;
}

// Setter for *pitchLimitRate* in rad / s
void ACROMapping::setPitchLimitRate(float newPitchLimitRate) noexcept
{
    pitchLimitRate = newPitchLimitRate;
}

// Setter for *rollLimitRate* in rad / s
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

/*
    Notes/Improvements:
    1. yaw and throttle priority
        here: roll/pitch -> 2% reserved for yaw -> scale RP if saturates -> add yaw -> scale RPY together if saturates-> clamp throttle to fit
        ardu: roll/pitch > throttle > yaw
        roll and pitch has the highest in both, but ardupilot sacrafices yaw over thruttle

        Ardupilot computes: throttle_thrust_best_rpy = -(rpy_low + rpy_high) / 2
        which is the throttle that perfectly centers roll/pitch contributions. Then yaw is added last, with a "yaw headroom" parameter, and if yaw doesn't fit it's scaled down

        Here we scale roll/pitch to make room for yaw. If pilot throttle is high it gets clamped down and yaw always gets added. 
        In ArduPilot, asking for full throttle while roll/pitching hard would drop yaw, not throttle.
        In practice, in a sharp turn, we would expect a slight altitude dip while ardupilot gets a tiny heading drift
        But this is compensation made due to very slow/little reaction of yaw compared to roll/pitch

    2. Yaw headroom can be made reactive to roll/pitch, currently is always set to 2%

    3. scale motor outputs dynamically based on battery status
        Ardupilot scaled moor outputs by battery_voltage / battery_voltage_resting (MOT_BAT_VOLT_*)
        so thrust stays constant as the battery drains. Without it, tuning may be less acurate as battey drains.
*/
void ACROMapping::motorMixer(const RCMotorControlMessage_t OUTPUT_CONTROL_MSG)
{
    // Roll, pitch, yaw in range [-1, 1], throttle in [0,1]
    float roll = OUTPUT_CONTROL_MSG.roll;
    float pitch = OUTPUT_CONTROL_MSG.pitch;
    float yaw = OUTPUT_CONTROL_MSG.yaw;
    float throttle = OUTPUT_CONTROL_MSG.throttle; 

    static const int8_t ROLL_SIGN[4] = { -1, 1, 1, -1};
    static const int8_t PITCH_SIGN[4] = { 1, -1, 1, -1};
    static const int8_t YAW_SIGN[4] = { 1, 1, -1, -1};

    const float YAW_HEADROOM = 0.02f;

    float max = 0.0f;
    float min = 0.0f;
    // Roll and Pitch
    for (int i = 0; i < 4; i++) {
        motorPercent[i] = roll * ROLL_SIGN[i] + pitch * PITCH_SIGN[i];
        max = fmaxf(max, motorPercent[i]);
        min = fminf(min, motorPercent[i]);
    }
    // Reduce roll and pitch if leaving no room for yaw
    float range = max - min;
    if (range > 1.0f - YAW_HEADROOM) {
        float scalingFactor = (1.0f - YAW_HEADROOM) / range;
        for (int i = 0; i < 4; i++) {
            motorPercent[i] *= scalingFactor;
        }
    }

    min = 0.0f;
    max = 0.0f;
    // Yaw
    for(int i = 0; i < 4; i++) {
        motorPercent[i] += yaw * YAW_SIGN[i];
        max = fmaxf(max, motorPercent[i]);
        min = fminf(min, motorPercent[i]);
    }
    range = max - min;
    // Reduce roll, pitch and yaw together if adding yaw saturates
    if (range > 1.0f ) {
        float scalingFactor = 1.0f / range;
        min = 0.0f;
        max = 0.0f;
        for (int i = 0; i < 4; i++) {
            motorPercent[i] *= scalingFactor;
            max = fmaxf(max, motorPercent[i]);
            min = fminf(min, motorPercent[i]);     
        }
    }

    // Throttle
    float minThrottle = fmaxf(-min, throttle);     // Add enough throttle to keep all motors >= 0
    float maxThrottle = 1.0f - max;
    throttle = fminf(minThrottle, maxThrottle);
    for(int i = 0; i < 4; i++) {
        motorPercent[i] += throttle;
    }

}

const float *ACROMapping::getMixedMotors() {
    return motorPercent;
}