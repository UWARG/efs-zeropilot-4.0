#include <cmath>
#include "attitude_manager.hpp"
#include "rc_motor_control.hpp"
#include "zp_params.hpp"
#include "motor_functions.hpp"

AttitudeManager::AttitudeManager(
    ISystemUtils *systemUtilsDriver,
    IGPS *gpsDriver,
    IIMU *imuDriver,
    IMessageQueue<RCMotorControlMessage_t> *amQueue,
    IMessageQueue<TMMessage_t> *tmQueue,
    IMessageQueue<char[100]> *smLoggerQueue,
    MotorGroupInstance_t *mainMotorGroup
) :
    systemUtilsDriver(systemUtilsDriver),
    gpsDriver(gpsDriver),
    imuDriver(imuDriver),
    amQueue(amQueue),
    tmQueue(tmQueue),
    smLoggerQueue(smLoggerQueue),
    activeCLAW(&manualCLAW),
    manualCLAW(),
    fbwaCLAW(AM_CONTROL_LOOP_PERIOD_S),
    controlMsg({50, 50, 50, 0, 0, 0, PlaneFlightMode_e::MANUAL}),
    droneState(DRONE_STATE_DEFAULT),
    currentFlightMode(PlaneFlightMode_e::MANUAL),
    mainMotorGroup(mainMotorGroup),
    armedFlag(false),
    setArmFlag(false),
    lastServoOutputs{0},
    amSchedulingCounter(0),
    noDataCount(0),
    failsafeTriggered(false),
    paramSetup(this) {

    paramSetup.loadAllParams();
    paramSetup.bindAllParamCallbacks();

    // Activate the activeCLAW
    activeCLAW->activateFlightMode();
}

void AttitudeManager::amUpdate() {

    amSchedulingCounter = (amSchedulingCounter + 1) % AM_SCHEDULING_RATE_HZ;

    // Send servo output raw data to telemetry manager
    if (amSchedulingCounter % (AM_SCHEDULING_RATE_HZ / AM_TELEMETRY_SERVO_OUTPUT_RAW_RATE_HZ) == 0) {
        sendServoOutputRawToTelemetryManager();
    }

    // Send IMU raw data to telemetry manager
    RawImu_t imuData = imuDriver->readRawData();
    ScaledImu_t scaledImuData = imuDriver->scaleIMUData(imuData);
    mahonyFilter.updateIMU(
        scaledImuData.xgyro,
        scaledImuData.ygyro,
        scaledImuData.zgyro,
        scaledImuData.xacc,
        scaledImuData.yacc,
        scaledImuData.zacc
    );
    Attitude_t attitude = mahonyFilter.getAttitudeRadians();
    droneState.roll = attitude.roll;
    droneState.pitch = attitude.pitch;
    droneState.yaw = attitude.yaw;

    if (amSchedulingCounter % (AM_SCHEDULING_RATE_HZ / AM_TELEMETRY_RAW_IMU_DATA_RATE_HZ) == 0) {
        sendRawIMUDataToTelemetryManager(imuData);
    }

    if (amSchedulingCounter % (AM_SCHEDULING_RATE_HZ / AM_TELEMETRY_ATTITUDE_DATA_RATE_HZ) == 0) {
        sendAttitudeDataToTelemetryManager(attitude);
    }

    // Send GPS data to telemetry manager
    GpsData_t gpsData = gpsDriver->readData();
    if (amSchedulingCounter % (AM_SCHEDULING_RATE_HZ / AM_TELEMETRY_GPS_DATA_RATE_HZ) == 0) {
        sendGPSDataToTelemetryManager(gpsData);
    }

    // Get data from Queue and motor outputs
    bool controlRes = getControlInputs(&controlMsg);
    
    if (controlRes != true) {
        ++noDataCount;

        if (noDataCount * AM_UPDATE_LOOP_DELAY_MS > ((ZP_PARAM::get(ZP_PARAM_ID::RC_FS_TIMEOUT)) * 1000)) {
            RCMotorControlMessage_t motorOutputs{0};
            motorOutputs.roll = 50;
            motorOutputs.pitch = 50;
            motorOutputs.yaw = 50;
            motorOutputs.throttle = 0;
            motorOutputs.flapAngle = 0;
            outputToMotors(motorOutputs);

            if (!failsafeTriggered) {
              char errorMsg[100] = "Failsafe triggered";
              smLoggerQueue->push(&errorMsg);
              failsafeTriggered = true;
            }

            return;
        }
    } else {
        noDataCount = 0;

        if (failsafeTriggered) {
          char errorMsg[100] = "Motor control restored";
          smLoggerQueue->push(&errorMsg);
          failsafeTriggered = false;
        }
    }

    // Update armedFlag and activateFlightMode() on rising edge
    if (controlMsg.arm != armedFlag) {
        setArmFlag = true;
        armedFlag = controlMsg.arm;
        if (armedFlag) {
            activeCLAW->activateFlightMode();
        }
    }

    // Update current flightmode if changed

    if (controlMsg.flightMode != currentFlightMode) {
        switch (controlMsg.flightMode) {
            #ifdef FIXED_WING
            case PlaneFlightMode_e::MANUAL:
                activeCLAW = &manualCLAW;
                break;
            case PlaneFlightMode_e::FBWA:
                activeCLAW = &fbwaCLAW;
                break;
            #endif

            #ifdef QUADCOPTER
            case CopterFlightMode_e::ACRO:
                activeCLAW = &manualCLAW;
                break;
            #endif
        }
        activeCLAW->activateFlightMode();
        currentFlightMode = controlMsg.flightMode;
    }

    // Run the active control law
    RCMotorControlMessage_t motorOutputs = activeCLAW->runControl(controlMsg, droneState);

    // Disarm logic
    if (!armedFlag) {
        motorOutputs.throttle = 0;
    }

    #ifdef QUADCOPTER
        motorMixer(motorOutputs);
    #endif

    // Output to motors
    outputToMotors(motorOutputs);

    setArmFlag = false;
}

void AttitudeManager::motorMixer(RCMotorControlMessage_t outputControlMsg) {
    
}

bool AttitudeManager::getControlInputs(RCMotorControlMessage_t *pControlMsg) {
    if (amQueue->count() == 0) {
        return false;
    }

    amQueue->get(pControlMsg);
    return true;
}

void AttitudeManager::outputToMotors(RCMotorControlMessage_t outputControlMsg) {
    #ifdef QUADCOPTER
    float motor_percent[4];
    motor_percent[0] = -outputControlMsg.roll + outputControlMsg.pitch;
    motor_percent[1] = outputControlMsg.roll - outputControlMsg.pitch;
    motor_percent[2] = outputControlMsg.roll + outputControlMsg.pitch;
    motor_percent[3] = -outputControlMsg.roll - outputControlMsg.pitch;

    bool disregard_yaw_flag = false;

    // Roll and Pitch
    float max_range = 0.0f;
    // max range
    for (int i = 0; i < 4; i++) {
        if (fabsf(motor_percent[i]) > max_range) {
            max_range = fabsf(motor_percent[i]);
        }
    }
    // scale down to [-1,1]
    if (max_range > 1) {
        float scaling_factor = 1 / max_range;
        for (int i = 0; i < 4; i++) {
            motor_percent[i] *= scaling_factor;
        }
    }

    // Throttle
    float min_throttle = 0.0f;
    for (int i = 0; i < 4; i++) {
        if (motor_percent[i] < 0 && fabsf(motor_percent[i]) > min_throttle) {
            min_throttle = fabsf(motor_percent[i]);
        }
    }
    if (outputControlMsg.throttle < min_throttle) { outputControlMsg.throttle = min_throttle; }

    float max_overshoot = 0.0f;
    for (int i = 0; i < 4; i++) {
        float overshoot = motor_percent[i] + outputControlMsg.throttle - 1;
        if (overshoot > max_overshoot) {
            max_overshoot = overshoot;
        }
    }

    if (max_overshoot > 0) {
        // Decrease throttle to fit in [0,1]
        for(int i = 0; i < 4; i++) {
            motor_percent[i] += outputControlMsg.throttle - max_overshoot;
        }
        disregard_yaw_flag = true;
    } else {
        // Throttle does not cause saturation
        for(int i = 0; i < 4; i++) {
            motor_percent[i] += outputControlMsg.throttle;
        }
    }

    // Yaw
    if (!disregard_yaw_flag) {
        float min_yaw_scale = 1.0f;
        int8_t yaw_signs[4] = { 1, 1, -1, -1 };
        for(int i = 0; i < 4; i++) {
            float yaw_contribution = outputControlMsg.yaw * yaw_signs[i]; // yaw contribution could be neg or pos
            if (yaw_contribution > 0) {
                // Saturates above 0 
                float yaw_scale = (1 - motor_percent[i]) / yaw_contribution;  
                if( yaw_scale < min_yaw_scale) {
                    min_yaw_scale = yaw_scale;
                }
            } else if (yaw_contribution < 0){
                // Saturates below 0
                float yaw_scale = motor_percent[i] / (-yaw_contribution);  
                if( yaw_scale < min_yaw_scale) {
                    min_yaw_scale = yaw_scale;
                }
            }
        }

        for (int i = 0; i < 4; i++) {
            motor_percent[i] += yaw_signs[i] * outputControlMsg.yaw * min_yaw_scale;
        }
    }
    #endif


    for (uint8_t i = 0; i < mainMotorGroup->motorCount; i++) {
        // Get current motor
        MotorInstance_t *motor = (mainMotorGroup->motors + i);

        float percent = 0.0f;
        #ifdef FIXED_WING
        // Extract percentage based on function
        switch (motor->function) {
            case MotorFunction_e::AILERON:
                percent = outputControlMsg.roll;
                break;
            case MotorFunction_e::ELEVATOR:
                percent = outputControlMsg.pitch;
                break;
            case MotorFunction_e::RUDDER:
                percent = outputControlMsg.yaw;
                break;
            case MotorFunction_e::THROTTLE:
                percent = outputControlMsg.throttle;
                break;
            case MotorFunction_e::FLAP:
                percent = outputControlMsg.flapAngle;
                break;
            case MotorFunction_e::GROUND_STEERING:
                percent = outputControlMsg.yaw;
                break;
            default:
                continue;
        }
        #endif

        #ifdef QUADCOPTER
        switch (motor->function) { 
            case MotorFunction_e::MOTOR_1:
                percent = motor_percent[0];
                break;
            case MotorFunction_e::MOTOR_2:
                percent = motor_percent[1];
                break;
            case MotorFunction_e::MOTOR_3:
                percent = motor_percent[2];
                break;
            case MotorFunction_e::MOTOR_4:
                percent = motor_percent[3];
                break;
            default:
                continue;  
        }
        #endif

        // Set cmd based on percent and trim, min, max
        uint32_t cmd = 0;
        #ifdef FIXED_WING
        if (percent <= 50.0f) {
            // Scale [0, 50] to [min, trim]
            cmd = motor->min + (percent / 50.0f) * (motor->trim - motor->min);
        } else {
            // Scale [50, 100] to [trim, max]
            cmd = motor->trim + ((percent - 50.0f) / 50.0f) * (motor->max - motor->trim);
        }
        #endif

        #ifdef QUADCOPTER

        #endif

        // Clamp cmd to [0, 100]
        if (cmd > 100) {
            cmd = 100;
        } else if (cmd < 0) {
            cmd = 0;
        }

        // Invert command if motor is inverted
        if (motor->isInverted) {
            cmd = 100 - cmd;
        }

        // Store for telemetry output
        lastServoOutputs[i] = 1000 + (cmd * 10); // Convert to microseconds for telemetry

        // set arm flag for throttle motors, only on arm/disarm edges
        if(setArmFlag) {
            bool armed = (motor->function == MotorFunction_e::THROTTLE) ? armedFlag : true;
            motor->motorInstance->setArm(armed);
        }

        // Send command to motor
        motor->motorInstance->set(cmd);
    }
}


void AttitudeManager::sendGPSDataToTelemetryManager(const GpsData_t &gpsData) {
    if (!gpsData.isNew) return;

    uint8_t fixType = (gpsData.numSatellites >= 4) ? 3 : 2; // 3 = 3D Fix, 2 = 2D Fix
    
    int32_t latE7 = static_cast<int32_t>(gpsData.latitude * 1e7f);
    int32_t lonE7 = static_cast<int32_t>(gpsData.longitude * 1e7f);
    int32_t altMM = static_cast<int32_t>(gpsData.altitude * 1000.0f);
    
    uint16_t velCmS = static_cast<uint16_t>(gpsData.groundSpeed);
    
    uint16_t cogCDeg = UINT16_MAX;
    if (gpsData.trackAngle != INVALID_TRACK_ANGLE) {
        float normalizedAngle = gpsData.trackAngle;
        while (normalizedAngle < 0) normalizedAngle += 360.0f;
        cogCDeg = static_cast<uint16_t>(normalizedAngle * 100.0f);
    }

    TMMessage_t gpsDataMsg = gpsRawDataPack(
        systemUtilsDriver->getCurrentTimestampMs(),
        fixType,
        latE7,
        lonE7,
        altMM,
        UINT16_MAX,  // eph: UINT16_MAX if unknown
        UINT16_MAX,  // epv: UINT16_MAX if unknown
        velCmS,
        cogCDeg,
        gpsData.numSatellites
    );

    tmQueue->push(&gpsDataMsg);
}

void AttitudeManager::sendRawIMUDataToTelemetryManager(const RawImu_t &imuData) {
    TMMessage_t imuDataMsg = rawImuDataPack(
        systemUtilsDriver->getCurrentTimestampMs(), // time_boot_ms
        imuData.xacc,
        imuData.yacc,
        imuData.zacc,
        imuData.xgyro,
        imuData.ygyro,
        imuData.zgyro
    );

    tmQueue->push(&imuDataMsg);
}

void AttitudeManager::sendAttitudeDataToTelemetryManager(const Attitude_t &attitude) {
    TMMessage_t attitudeDataMsg = attitudeDataPack(
        systemUtilsDriver->getCurrentTimestampMs(), // time_boot_ms
        attitude.roll,
        attitude.pitch,
        attitude.yaw
    );

    tmQueue->push(&attitudeDataMsg);
}

void AttitudeManager::sendServoOutputRawToTelemetryManager() {
    TMMessage_t servoOutputMsg = servoOutputRawPack(
        systemUtilsDriver->getCurrentTimestampMs(), // time_boot_ms
        0, // port hardcoded to 0 since we are using MAVLink2 with 16 servo outputs in one message
        lastServoOutputs
    );

    tmQueue->push(&servoOutputMsg);
}
