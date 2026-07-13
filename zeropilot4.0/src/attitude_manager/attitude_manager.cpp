#include "attitude_manager.hpp"
#include "rc_motor_control.hpp"
#include "zp_params.hpp"
#include "motor_functions.hpp"
#include "unit_conversions.hpp"

AttitudeManager::AttitudeManager(
    ISystemUtils *systemUtilsDriver,
    IGPS *gpsDriver,
    IIMU *imuDriver,
    IFFT *fftDriver,
    IMessageQueue<RCMotorControlMessage_t> *amQueue,
    IMessageQueue<TMMessage_t> *tmQueue,
    IMessageQueue<char[100]> *smLoggerQueue,
    MotorGroupInstance_t *mainMotorGroup
) :
    systemUtilsDriver(systemUtilsDriver),
    gpsDriver(gpsDriver),
    imuDriver(imuDriver),
    harmonicNotchFilter(systemUtilsDriver, fftDriver),
    amQueue(amQueue),
    tmQueue(tmQueue),
    smLoggerQueue(smLoggerQueue),
    #ifdef PLANE
    activeCLAW(&manualCLAW),
    manualCLAW(),
    fbwaCLAW(AM_CONTROL_LOOP_PERIOD_S),
    controlMsg({50, 50, 50, 0, 0, 0, FlightMode_e::MANUAL}),
    currentFlightMode(FlightMode_e::MANUAL),
    #endif
    #ifdef QUADCOPTER
    activeCLAW(&stabilizeCLAW),
    acroCLAW(AM_CONTROL_LOOP_PERIOD_S),
    stabilizeCLAW(AM_CONTROL_LOOP_PERIOD_S, acroCLAW),
    controlMsg({50, 50, 50, 0, 0, FlightMode_e::STABILIZE}),
    currentFlightMode(FlightMode_e::STABILIZE),
    #endif
    droneState(DRONE_STATE_DEFAULT),
    mainMotorGroup(mainMotorGroup),
    armedFlag(false),
    setArmFlag(false),
    lastServoOutputs{0},
    amSchedulingCounter(0),
    noDataCount(0),
    failsafeTriggered(false),
    lastTimestamp(0),
    haveLastImuTimestamp(false),
    profilerId(0),
    paramSetup(this) {
        paramSetup.loadAllParams();
        paramSetup.bindAllParamCallbacks();

        harmonicNotchConfig.sampleFreqHz = imuDriver->getODRHz();
        harmonicNotchFilter.init(harmonicNotchConfig);

        // Activate the activeCLAW
        activeCLAW->activateFlightMode();

        systemUtilsDriver->profilerRegister("AM", &profilerId);
}

void AttitudeManager::amUpdate() {

    systemUtilsDriver->profilerBegin(profilerId);

    amSchedulingCounter = (amSchedulingCounter + 1) % AM_SCHEDULING_RATE_HZ;

    // Send servo output raw data to telemetry manager
    if (amSchedulingCounter % (AM_SCHEDULING_RATE_HZ / AM_TELEMETRY_SERVO_OUTPUT_RAW_RATE_HZ) == 0) {
        sendServoOutputRawToTelemetryManager();
    }

    // Send IMU raw data to telemetry manager
    RawImuBatch_t imuData = imuDriver->readRawData();
    ScaledImuBatch_t scaledImuData = imuDriver->scaleIMUData(imuData);
    for (int i = 0; i < scaledImuData.count; i++) {
        if (scaledImuData.data[i].imuId == 0) { // Only feed one IMU's data for FFT sampling as we need a continuous time stream.
            harmonicNotchFilter.pushSample(scaledImuData.data[i].xgyro, scaledImuData.data[i].ygyro, scaledImuData.data[i].zgyro);
        }
        // By nature of FFT algorithm there is a correction latency dependant on the FFT length and sample rate.
        harmonicNotchFilter.apply(scaledImuData.data[i].xgyro, scaledImuData.data[i].ygyro, scaledImuData.data[i].zgyro);
        
        /**
         * We use uint16_t instead of uint32_t as single IMU logic relies on uint16_t wraparound
         * and the delta for double IMU will be necessarily less than uint16_t max value.
         */
        uint16_t deltaTicks = scaledImuData.data[i].timestamp - lastTimestamp;

        lastTimestamp = scaledImuData.data[i].timestamp;

        // Make lastTimestamp hold a real timestamp the first iteration
        if (!haveLastImuTimestamp) {
            haveLastImuTimestamp = true;
            continue;
        }

        float dt = deltaTicks * TIMESTAMP_RESOLUTION;

        mahonyFilter.updateIMU(
            scaledImuData.data[i].xgyro,
            scaledImuData.data[i].ygyro,
            scaledImuData.data[i].zgyro,
            scaledImuData.data[i].xacc,
            scaledImuData.data[i].yacc,
            scaledImuData.data[i].zacc,
            dt
        );
    }
    Attitude_t attitude = mahonyFilter.getAttitudeRadians();
    droneState.roll = attitude.roll;
    droneState.pitch = attitude.pitch;
    droneState.yaw = attitude.yaw;
    uint8_t scaledImuCount = scaledImuData.count;
    if (scaledImuCount > 0) { // Use most recent sample in the batch for rates
        droneState.rollRate = scaledImuData.data[scaledImuCount - 1].xgyro * ZP_UNITS::DEG_TO_RAD;
        droneState.pitchRate = scaledImuData.data[scaledImuCount - 1].ygyro * ZP_UNITS::DEG_TO_RAD;
        droneState.yawRate = scaledImuData.data[scaledImuCount - 1].zgyro * ZP_UNITS::DEG_TO_RAD;
    }

    if (amSchedulingCounter % (AM_SCHEDULING_RATE_HZ / AM_TELEMETRY_RAW_IMU_DATA_RATE_HZ) == 0) {
        if (imuData.count > 0) { sendRawIMUDataToTelemetryManager(imuData.data[imuData.count - 1]); } // Send the last packed of IMU data 
    }

    if (amSchedulingCounter % (AM_SCHEDULING_RATE_HZ / AM_TELEMETRY_ATTITUDE_DATA_RATE_HZ) == 0) {
        sendAttitudeDataToTelemetryManager(attitude);
    }

    // Send GPS data to telemetry manager
    if (amSchedulingCounter % (AM_SCHEDULING_RATE_HZ / AM_TELEMETRY_GPS_DATA_RATE_HZ) == 0) {
        GpsData_t gpsData = gpsDriver->readData();
        sendGPSDataToTelemetryManager(gpsData);
    }

    // Get data from Queue and motor outputs
    bool controlRes = getControlInputs(&controlMsg);
    
    if (controlRes != true) {
        ++noDataCount;

        if (noDataCount * AM_UPDATE_LOOP_DELAY_MS > ((ZP_PARAM::get(ZP_PARAM_ID::RC_FS_TIMEOUT)) * 1000)) {
            RCMotorControlMessage_t motorOutputs{0};

            #ifdef PLANE
            motorOutputs.roll = 50;
            motorOutputs.pitch = 50;
            motorOutputs.yaw = 50;
            motorOutputs.throttle = 0;
            motorOutputs.flapAngle = 0;
            #endif

            #ifdef QUADCOPTER
            motorOutputs.roll = 0;
            motorOutputs.pitch = 0;
            motorOutputs.yaw = 0;
            motorOutputs.throttle = 0;      
            #endif
            
            if (!failsafeTriggered) {
                char errorMsg[100] = "Failsafe triggered";
                smLoggerQueue->push(&errorMsg);
                failsafeTriggered = true;
            }
            
            outputToMotors(motorOutputs);

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

            #ifdef PLANE
            case FlightMode_e::MANUAL:
                activeCLAW = &manualCLAW;
                break;
            case FlightMode_e::FBWA:
                activeCLAW = &fbwaCLAW;
                break;
            #endif

            #ifdef QUADCOPTER
            case FlightMode_e::ACRO:
                activeCLAW = &acroCLAW;
                break;
            case FlightMode_e::STABILIZE:
                activeCLAW = &stabilizeCLAW;
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

        #ifdef QUADCOPTER
        motorOutputs.pitch = 0;
        motorOutputs.roll = 0;
        motorOutputs.yaw = 0;
        #endif

    }

    // Output to motors
    outputToMotors(motorOutputs);

    setArmFlag = false;
    
    systemUtilsDriver->profilerEnd(profilerId);
}

bool AttitudeManager::getControlInputs(RCMotorControlMessage_t *pControlMsg) {
    if (amQueue->count() == 0) {
        return false;
    }

    amQueue->get(pControlMsg);
    return true;
}

void AttitudeManager::outputToMotors(const RCMotorControlMessage_t outputControlMsg) {

    #ifdef PLANE
        MotorMixing::fixedWingMoterMixer(outputControlMsg, mainMotorGroup, motorPercent);
    #endif

    #ifdef QUADCOPTER
        MotorMixing::quadMotorMixer(outputControlMsg, mainMotorGroup, motorPercent);
    #endif

    for (uint8_t i = 0; i < mainMotorGroup->motorCount; i++) {
        // Get current motor
        MotorInstance_t *motor = (mainMotorGroup->motors + i);

        if (motor->function == MotorFunction_e::DISABLED || motor->function == MotorFunction_e::GPIO) {
            continue;
        }

        float percent = motorPercent[i];

        #ifdef QUADCOPTER
        if (!armedFlag || failsafeTriggered) {
            percent = 0;
        }
        #endif

        uint32_t cmd = 0;

        #ifdef PLANE
        // Set cmd based on percent and trim, min, max
        if (percent <= 50.0f) {
            // Scale [0, 50] to [min, trim]
            cmd = motor->min + (percent / 50.0f) * (motor->trim - motor->min);
        } else {
            // Scale [50, 100] to [trim, max]
            cmd = motor->trim + ((percent - 50.0f) / 50.0f) * (motor->max - motor->trim);
        }

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
        #endif

        #ifdef QUADCOPTER
        cmd = percent * 100;
        if (cmd > 100.0f) cmd = 100.0f; 
        else if (cmd < 0.0f) cmd = 0.0f; 
        #endif

        // Store for telemetry output
        lastServoOutputs[i] = 1000 + (cmd * 10); // Convert to microseconds for telemetry

        // Set arm flag for throttle motors, only on arm/disarm edges
        if (setArmFlag) {

            #ifdef PLANE
            bool armed = (motor->function == MotorFunction_e::THROTTLE) ? armedFlag : true;
            #endif
            
            #ifdef QUADCOPTER
            bool armed = (motor->function == MotorFunction_e::MOTOR_1 || motor->function == MotorFunction_e::MOTOR_2 
                || motor->function == MotorFunction_e::MOTOR_3 || motor->function == MotorFunction_e::MOTOR_4) ? armedFlag : true;  
            #endif

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
