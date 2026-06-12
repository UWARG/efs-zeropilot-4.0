#include "motor_mixing.hpp"

#ifdef FIXED_WING
void MotorMixing::fixedWingMoterMixer(const RCMotorControlMessage_t OUTPUT_CONTROL_MSG,  MotorGroupInstance_t *mainMotorGroup, float* motorPercent) {
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
}
#endif

#ifdef QUADCOPTER
void MotorMixing::quadMotorMixer(const RCMotorControlMessage_t OUTPUT_CONTROL_MSG,  MotorGroupInstance_t *mainMotorGroup, float* motorPercent) {
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
   static constexpr float F = 0.7071067811865476f;
   static constexpr float ROLL_SIGN[4] = { -F, F, F, -F};
   static constexpr float PITCH_SIGN[4] = { F, -F, F, -F};
   static constexpr float YAW_SIGN[4] = { 1, 1, -1, -1};
   
   float mixed[4] = {0};
   #if 0
   const float YAW_HEADROOM = 0.02f;

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
    #endif
    static constexpr uint8_t NUM_MOTORS = 4;
    static constexpr float YAW_HEADROOM = 0.2f;

    // Ensure the maximum average throttle across the 4 motors are at least the throttle commanded and never exceeds the set max
    float throttleAvgMax = throttle;

    // The optimal throttle that gives the best symmetric room for rpy (equal room for above and below)
    // 0.5 is the best but take the commanded throttle if its lower than 0.5, and take 0.5 if its higher, so the drone wont elevate when little throttle is commanded
    float idealThrottle = fminf(0.5f, throttleAvgMax);

    // Add roll and pitch, while finding the yaw allowed at the same time
    float yaw_allowed = 1.0f;
    for (int i = 0; i < NUM_MOTORS; i++) {
        mixed[i] = roll * ROLL_SIGN[i] + pitch * PITCH_SIGN[i];

        float predictedMotorThrust = idealThrottle + mixed[i]; 
        float motorRoom = (yaw * YAW_SIGN[i] >= 0) 
                            ? 1.0f - predictedMotorThrust // Yaw is added to overall thrust
                            : predictedMotorThrust; // Yaw is subtracted from overall thrust
        yaw_allowed = fminf(yaw_allowed, fmaxf(motorRoom, 0)); // fmaxf(motorRoom, 0) so motorRoom being negative means 0 yaw allowed
    }

    // Clip yaw 
    yaw_allowed = fmaxf(YAW_HEADROOM, yaw_allowed); // Yaw is at least the headroom reserved
    if (fabsf(yaw) > yaw_allowed) {
        yaw = fmaxf(-yaw_allowed, fminf(yaw, yaw_allowed));
    }
    
    // Add yaw in and track the range rpy spans
    float rpy_max = 0.0f;
    float rpy_min = 1.0f;
    for (int i = 0; i < NUM_MOTORS; i++) {
        mixed[i] += yaw * YAW_SIGN[i];
        rpy_max = fmaxf(rpy_max, mixed[i]);
        rpy_min = fminf(rpy_min, mixed[i]);
    }

    // Scale rpy span
    float rpy_scale = 1.0f;
    if (rpy_max - rpy_min >= 1.0f) {
        // Total rpy span exceeds 1.0, scale everything down
        rpy_scale = 1.0f / (rpy_max - rpy_min);
    }
    if (throttleAvgMax + rpy_min < 0.0f) {
        // The lowest value motor still below 0 after applying the max allowed collective thrust 
        rpy_scale = fminf(rpy_scale, fabsf(throttleAvgMax / rpy_min)); // Scale down rpy together to make the lowest motor fit
    }
    rpy_max *= rpy_scale;
    rpy_min *= rpy_scale;

    // Collective throttle that prevents the lowest motor from going negative
    float minThrottle = fabsf(rpy_min); 
    // The amount needed to shift up from minThrottle to match the commanded throttle
    float throttle_adj = throttle - minThrottle;
    // Calculate throttle to add
    if (rpy_scale < 1.0f) {
        // The rpy already saturated so no room for throttle
        throttle_adj = 0.0f;
    } else if (throttle_adj < 0.0f) {
        // Wants less throttle than minThrottle, will make some motor negative, drop it(not decreasing throttle)
        throttle_adj = 0.0f;
    } else if (throttle_adj > (1.0f - minThrottle - rpy_max)) {
        // Wants more throttle but has no room, cap it to the available room
        throttle_adj = 1.0f - minThrottle - rpy_max;
    }
    float finalThrottle = minThrottle + throttle_adj;

    // Final output
    for (int i = 0; i < NUM_MOTORS; i++) {
        mixed[i] = finalThrottle + rpy_scale * mixed[i];
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
}
#endif

