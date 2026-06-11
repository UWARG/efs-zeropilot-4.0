#include "motor_mixing.hpp"


float *MotorMixing::fixedWingMoterMixer(const RCMotorControlMessage_t OUTPUT_CONTROL_MSG,  MotorGroupInstance_t *mainMotorGroup, float* motorPercent) {
    for (uint8_t i = 0; i < mainMotorGroup->motorCount; i++) {
        switch (mainMotorGroup->motors[i].function) {
            case MotorFunction_e::AILERON: 
                motorPercent[i] = OUTPUT_CONTROL_MSG.roll;
                break;
            case MotorFunction_e::ELEVATOR:
                motorPercent[i] = OUTPUT_CONTROL_MSG.pitch;
                break;
            case MotorFunction_e::RUDDER:
                motorPercent[i] = OUTPUT_CONTROL_MSG.yaw;
                break;
            case MotorFunction_e::THROTTLE:
                motorPercent[i] = OUTPUT_CONTROL_MSG.throttle;
                break;
            case MotorFunction_e::FLAP:
                motorPercent[i] = OUTPUT_CONTROL_MSG.flapAngle;
                break;
            case MotorFunction_e::GROUND_STEERING: 
                motorPercent[i] = OUTPUT_CONTROL_MSG.yaw;
                break;
            default: 
                motorPercent[i] = 0.0f;
                break;
        }
    }
    return motorPercent;
}

float *MotorMixing::quadMotorMixer(const RCMotorControlMessage_t OUTPUT_CONTROL_MSG,  MotorGroupInstance_t *mainMotorGroup, float* motorPercent) {
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

    // Roll, pitch, yaw in range [-1, 1], throttle in [0,1]
    float roll = OUTPUT_CONTROL_MSG.roll;
    float pitch = OUTPUT_CONTROL_MSG.pitch;
    float yaw = OUTPUT_CONTROL_MSG.yaw;
    float throttle = OUTPUT_CONTROL_MSG.throttle; 

    static const int8_t ROLL_SIGN[4] = { -1, 1, 1, -1};
    static const int8_t PITCH_SIGN[4] = { 1, -1, 1, -1};
    static const int8_t YAW_SIGN[4] = { 1, 1, -1, -1};

    const float YAW_HEADROOM = 0.02f;

    float mixed[4] = {0};
    float max = 0.0f;
    float min = 0.0f;
    // Roll and Pitch
    for (int i = 0; i < 4; i++) {
        mixed[i] = roll * ROLL_SIGN[i] + pitch * PITCH_SIGN[i];
        max = fmaxf(max, mixed[i]);
        min = fminf(min, mixed[i]);
    }
    // Reduce roll and pitch if leaving no room for yaw
    float range = max - min;
    if (range > 1.0f - YAW_HEADROOM) {
        float scalingFactor = (1.0f - YAW_HEADROOM) / range;
        for (int i = 0; i < 4; i++) {
            mixed[i] *= scalingFactor;
        }
    }

    min = 0.0f;
    max = 0.0f;
    // Yaw
    for(int i = 0; i < 4; i++) {
        mixed[i] += yaw * YAW_SIGN[i];
        max = fmaxf(max, mixed[i]);
        min = fminf(min, mixed[i]);
    }
    range = max - min;
    // Reduce roll, pitch and yaw together if adding yaw saturates
    if (range > 1.0f ) {
        float scalingFactor = 1.0f / range;
        min = 0.0f;
        max = 0.0f;
        for (int i = 0; i < 4; i++) {
            mixed[i] *= scalingFactor;
            max = fmaxf(max, mixed[i]);
            min = fminf(min, mixed[i]);
        }
    }

    // Throttle
    float minThrottle = fmaxf(-min, throttle);     // Add enough throttle to keep all motors >= 0
    float maxThrottle = 1.0f - max;
    throttle = fminf(minThrottle, maxThrottle);
    for(int i = 0; i < 4; i++) {
        mixed[i] += throttle;
    }

    // Place mixed motor outputs into physical channels by function
    for (uint8_t i = 0; i < mainMotorGroup->motorCount; i++) {
        switch (mainMotorGroup->motors[i].function) {
            case MotorFunction_e::MOTOR_1: 
                motorPercent[i] = mixed[0]; 
                break;
            case MotorFunction_e::MOTOR_2: 
                motorPercent[i] = mixed[1]; 
                break;
            case MotorFunction_e::MOTOR_3: 
                motorPercent[i] = mixed[2]; 
                break;
            case MotorFunction_e::MOTOR_4: 
                motorPercent[i] = mixed[3]; 
                break;
            default:
                motorPercent[i] = 0.0f;
                break;
        }
    }
    return motorPercent;
}

