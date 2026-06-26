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
    static constexpr uint8_t NUM_MOTORS = 4;
    static constexpr float YAW_HEADROOM = 0.2f;

    // Ensure the maximum average throttle across the 4 motors are at least the throttle commanded and never exceeds the set max
    float throttleAvgMax = throttle;

    // The optimal throttle that gives the best symmetric room for rpy (equal room for above and below)
    // 0.5 is the best but take the commanded throttle if its lower than 0.5, and take 0.5 if its higher, so the drone wont elevate when little throttle is commanded
    float idealThrottle = fminf(0.5f, throttleAvgMax);

    // Add roll and pitch, while finding the yaw allowed at the same time
    float yawAllowed = 1.0f;
    for (int i = 0; i < NUM_MOTORS; i++) {
        mixed[i] = roll * ROLL_SIGN[i] + pitch * PITCH_SIGN[i];

        float predictedMotorThrust = idealThrottle + mixed[i]; 
        float motorRoom = (yaw * YAW_SIGN[i] >= 0) 
                            ? 1.0f - predictedMotorThrust // Yaw is added to overall thrust
                            : predictedMotorThrust; // Yaw is subtracted from overall thrust
        yawAllowed = fminf(yawAllowed, fmaxf(motorRoom, 0)); // fmaxf(motorRoom, 0) so motorRoom being negative means 0 yaw allowed
    }

    // Clip yaw 
    yawAllowed = fmaxf(YAW_HEADROOM, yawAllowed); // Yaw is at least the headroom reserved
    if (fabsf(yaw) > yawAllowed) {
        yaw = fmaxf(-yawAllowed, fminf(yaw, yawAllowed));
    }

    // Add yaw in and track the range rpy spans
    float rpyMax = 0.0f;
    float rpyMin = 1.0f;
    for (int i = 0; i < NUM_MOTORS; i++) {
        mixed[i] += yaw * YAW_SIGN[i];
        rpyMax = fmaxf(rpyMax, mixed[i]);
        rpyMin = fminf(rpyMin, mixed[i]);
    }

    // Scale rpy span
    float rpyScale = 1.0f;
    if (rpyMax - rpyMin >= 1.0f) {
        // Total rpy span exceeds 1.0, scale everything down
        rpyScale = 1.0f / (rpyMax - rpyMin);
    }
    if (throttleAvgMax + rpyMin < 0.0f) {
        // The lowest value motor still below 0 after applying the max allowed collective thrust 
        rpyScale = fminf(rpyScale, fabsf(throttleAvgMax / rpyMin)); // Scale down rpy together to make the lowest motor fit
    }
    rpyMax *= rpyScale;
    rpyMin *= rpyScale;

    // Collective throttle that prevents the lowest motor from going negative
    float minThrottle = fabsf(rpyMin); 
    // The amount needed to shift up from minThrottle to match the commanded throttle
    float throttleAdj = throttle - minThrottle;
    // Calculate throttle to add
    if (rpyScale < 1.0f) {
        // The rpy already saturated so no room for throttle
        throttleAdj = 0.0f;
    } else if (throttleAdj < 0.0f) {
        // Wants less throttle than minThrottle, will make some motor negative, drop it(not decreasing throttle)
        throttleAdj = 0.0f;
    } else if (throttleAdj > (1.0f - minThrottle - rpyMax)) {
        // Wants more throttle but has no room, cap it to the available room
        throttleAdj = 1.0f - minThrottle - rpyMax;
    }
    float finalThrottle = minThrottle + throttleAdj;

    // Final output
    for (int i = 0; i < NUM_MOTORS; i++) {
        mixed[i] = finalThrottle + rpyScale * mixed[i];
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

