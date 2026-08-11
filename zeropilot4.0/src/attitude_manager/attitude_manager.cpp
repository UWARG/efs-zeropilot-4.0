#include "attitude_manager.hpp"
#include "rc_motor_control.hpp"
#include "zp_params.hpp"
#include "motor_functions.hpp"
#include "unit_conversions.hpp"
#include <limits>

AttitudeManager::AttitudeManager(
    ISystemUtils *systemUtilsDriver,
    IMathUtils *mathUtilsDriver,
    IGPS *gpsDriver,
    IIMU *imuDriver,
    IFFT *fftDriver,
    IRangefinder *rangefinderDriver,
    IBarometer *barometerDriver,
    IMessageQueue<RCMotorControlMessage_t> *amQueue,
    IMessageQueue<TMMessage_t> *tmQueue,
    IMessageQueue<char[100]> *smLoggerQueue,
    MotorGroupInstance_t *mainMotorGroup
) :
    systemUtilsDriver(systemUtilsDriver),
    gpsDriver(gpsDriver),
    imuDriver(imuDriver),
    rangefinderDriver(rangefinderDriver),
    barometerDriver(barometerDriver),
    harmonicNotchFilter(mathUtilsDriver, fftDriver),
    // ekf(mathUtilsDriver),
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
    groundIdlePrev(false),
    lastTimestamp(0),
    haveLastImuTimestamp(false),
    profilerId(0),
    paramSetup(this) {
        paramSetup.loadAllParams();
        paramSetup.bindAllParamCallbacks();

        harmonicNotchConfig.sampleFreqHz = imuDriver->getODRHz();
        harmonicNotchFilter.init(harmonicNotchConfig);

        /* TODO: Uncomment once using EKF
        // Init the EKF
        AHRSEKF::Config ekfCfg = {
            .gyroCov = 4.78e-6f,
            .accelCov = 9.41e-4f,
            .magCov = 3.6e-5f,
            .gyroBiasCov = 1.0e-6f,
            .accelBiasCov = 0.0f,
            .accelGateThreshold = std::numeric_limits<float>::max(), // Turning off gating bc if start position is not leveled, then gating prevents convergence
            .magGateThreshold = 16.3f,
            .pInitAtt = 1e-2f,
            .pInitBiasGyro = 1e-3f,
            .pInitBiasAccel = 0.0f, // Assume P is a diagonal matrix
            .gravityInertial = {0, 0, 9.81f},
            .magInertial = {1, 0, 0}
        };
        float initGyro[3] = {0.0f, 0.0f, 0.0f};
        float initAccel[3] = {0.0f, 0.0f, -9.81f};
        float initMag[3] = {1.0f, 0.0f, 0.0f};
        float initQuat[4] = {1.0f, 0.0f, 0.0f, 0.0f};
        ekf.init(initGyro, initAccel, initMag, initQuat, ekfCfg);
        */

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

    // Read barometer data
    BaroData_t baroData;
    barometerDriver->readData(baroData);

    // Send scaled pressure data to TM
    if (amSchedulingCounter % (AM_SCHEDULING_RATE_HZ / AM_TELEMETRY_SCALED_PRESSURE_DATA_RATE_HZ) == 0) {
        sendPressureDataToTelemetryManager(baroData);
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
       
        /* TODO: Uncomment once using EKF
        if (scaledImuData.data[i].imuId != 0) continue; // Only use IMU0 for EKF
        */

        /*
        We use uint16_t instead of uint32_t as single IMU logic relies on uint16_t wraparound
        and the delta for double IMU will be necessarily less than uint16_t max value.
        */
        
        uint16_t deltaTicks = scaledImuData.data[i].timestamp - lastTimestamp;

        lastTimestamp = scaledImuData.data[i].timestamp;

        // Make lastTimestamp hold a real timestamp the first iteration
        if (!haveLastImuTimestamp) {
            haveLastImuTimestamp = true;
            continue;
        }

        GyroBias_t startupGyroBias = imuDriver->getGyroStartupBias(scaledImuData.data[i].imuId);
        droneState.rollRate = scaledImuData.data[i].xgyro - startupGyroBias.x;
        droneState.pitchRate = scaledImuData.data[i].ygyro - startupGyroBias.y;
        droneState.yawRate = scaledImuData.data[i].zgyro - startupGyroBias.z;

        float dt = deltaTicks * TIMESTAMP_RESOLUTION;
        
        mahonyFilter.updateIMU(
            scaledImuData.data[i].xgyro - startupGyroBias.x,
            scaledImuData.data[i].ygyro - startupGyroBias.y,
            scaledImuData.data[i].zgyro - startupGyroBias.z,
            scaledImuData.data[i].xacc,
            scaledImuData.data[i].yacc,
            scaledImuData.data[i].zacc,
            dt
        );

        /* TODO: Uncomment once using EKF
        float gyro[3] = {
            scaledImuData.data[i].xgyro,
            scaledImuData.data[i].ygyro,
            scaledImuData.data[i].zgyro
        };
        float accel[3] = {
            scaledImuData.data[i].xacc, 
            scaledImuData.data[i].yacc, 
            scaledImuData.data[i].zacc
        };

        ekf.stateExtrapolation(gyro, dt);
        if (amSchedulingCounter % 10 == 0) { // Correct accel once for every 10 gyro updates
            ekf.correctionAccelerometer(accel);
        }
        GyroBias_t gyroBias = ekf.getGyroBias();

        break; // for now only use one imu message per am loop
        */
    }

    Attitude_t attitude = mahonyFilter.getAttitudeRadians();
    droneState.roll = attitude.roll;
    droneState.pitch = attitude.pitch;
    droneState.yaw = attitude.yaw;

    if (amSchedulingCounter % (AM_SCHEDULING_RATE_HZ / AM_TELEMETRY_RAW_IMU_DATA_RATE_HZ) == 0) {
        if (imuData.count > 0) { sendRawIMUDataToTelemetryManager(imuData.data[imuData.count - 1]); } // Send the last packed of IMU data 
    }

    if (amSchedulingCounter % (AM_SCHEDULING_RATE_HZ / AM_TELEMETRY_ATTITUDE_DATA_RATE_HZ) == 0) {
        sendAttitudeDataToTelemetryManager(attitude);
    }

    // Get GPS data
    GpsData_t gpsData = gpsDriver->readData();
    if (gpsData.isNew) {
        lastValidGps = gpsData;
    }
    
    // Send GPS data to telemetry manager
    if (amSchedulingCounter % (AM_SCHEDULING_RATE_HZ / AM_TELEMETRY_GPS_DATA_RATE_HZ) == 0) {
        if (lastValidGps.isNew) {
            sendGPSDataToTelemetryManager(lastValidGps);
            lastValidGps.isNew = false; // Mark as sent to telemetry manager, so if no new GPS data is valid the same data is not sent again
        }
    }

    // Get rangefinder data
    RangefinderData_t rangefinderData = {};
    if (rangefinderDriver != nullptr) {
        rangefinderData = rangefinderDriver->readData();
        if (rangefinderData.isNew) {
            lastNewRangefinderData = rangefinderData;
        }
    }

    // Send rangefinder data to telemetry manager
    if (amSchedulingCounter % (AM_SCHEDULING_RATE_HZ / AM_TELEMETRY_DISTANCE_SENSOR_DATA_RATE_HZ) == 0) {
        if (lastNewRangefinderData.isNew) {
            sendRangefinderDataToTelemetryManager(lastNewRangefinderData);
            lastNewRangefinderData.isNew = false; // Mark as sent to telemetry manager, so if no new rangefinder data is valid the same data is not sent again
        }
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
            
            outputToMotors(motorOutputs, false);

            systemUtilsDriver->profilerEnd(profilerId);
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

    bool groundIdle = false;
    #ifdef QUADCOPTER
    groundIdle = armedFlag && !failsafeTriggered && ((controlMsg.throttle / 100.0f) <= MOT_GND_IDLE_THR);
    if (groundIdlePrev && !groundIdle) {
        activeCLAW->activateFlightMode(); // Clean PID state on ground idle exit
    }
    groundIdlePrev = groundIdle;
    #endif
    #ifdef PLANE
    groundIdle = false; // Hardcode to false for plane, they dont have a ground idle mode
    #endif

    // Run the active control law (skip while armed but grounded idle)
    RCMotorControlMessage_t motorOutputs = groundIdle ? controlMsg : activeCLAW->runControl(controlMsg, droneState);

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
    outputToMotors(motorOutputs, groundIdle);

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

void AttitudeManager::outputToMotors(const RCMotorControlMessage_t outputControlMsg, bool groundIdle) {

    #ifdef PLANE
        MotorMixing::fixedWingMoterMixer(outputControlMsg, mainMotorGroup, motorPercent);
    #endif

    #ifdef QUADCOPTER
        if (groundIdle) {
            MotorMixing::quadGroundIdle(mainMotorGroup, motorPercent, motSpinArm);
        } else {
            MotorMixing::quadMotorMixer(outputControlMsg, mainMotorGroup, motorPercent);
        }
    #endif

    for (uint8_t i = 0; i < mainMotorGroup->motorCount; i++) {
        // Get current motor
        MotorInstance_t *motor = (mainMotorGroup->motors + i);

        if (motor->function == MotorFunction_e::DISABLED || motor->function == MotorFunction_e::GPIO) {
            continue;
        }

        float percent = motorPercent[i];

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
        #endif
        
        #ifdef QUADCOPTER
        if (!armedFlag || failsafeTriggered) {
            percent = 0;
        } else if (!groundIdle) {
            percent = motSpinMin + percent * (motSpinMax - motSpinMin);
        }
        cmd = percent * 100;
        #endif

        // Clamp cmd to [0, 100], safety check
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

void AttitudeManager::sendPressureDataToTelemetryManager(const BaroData_t &baroData) {
    TMMessage_t pressureDataMsg = scaledPressurePack(
        systemUtilsDriver->getCurrentTimestampMs(), // time_boot_ms
        baroData.pressureKPa,
        0,
        baroData.temperatureC,
        0
    );

    tmQueue->push(&pressureDataMsg);
}

void AttitudeManager::sendRangefinderDataToTelemetryManager(const RangefinderData_t &rangefinderData) {
    float invalidQuaternion[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    TMMessage_t rangefinderDataMsg = distanceSensorDataPack(
        systemUtilsDriver->getCurrentTimestampMs(), // time_boot_ms
        ZP_PARAM::get(ZP_PARAM_ID::RNGFND_MIN),
        ZP_PARAM::get(ZP_PARAM_ID::RNGFND_MAX),
        rangefinderData.distance,
        1, // id
        0.01f, // covariance from datasheet of TF02
        0, // horizontalFov: invalid
        0, // verticalFov: invalid
        invalidQuaternion,
        rangefinderData.isValid ? ((rangefinderData.signalStrength / 65535.0f) * 100.0f) : 1 // % signalQuality, 1 = no signal
    );

    tmQueue->push(&rangefinderDataMsg);
}

void AttitudeManager::sendServoOutputRawToTelemetryManager() {
    TMMessage_t servoOutputMsg = servoOutputRawPack(
        systemUtilsDriver->getCurrentTimestampMs(), // time_boot_ms
        0, // port hardcoded to 0 since we are using MAVLink2 with 16 servo outputs in one message
        lastServoOutputs
    );

    tmQueue->push(&servoOutputMsg);
}
