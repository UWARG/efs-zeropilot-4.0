#include "attitude_manager.hpp"
#include "rc_motor_control.hpp"

AttitudeManager::AttitudeManager(
    ISystemUtils *systemUtilsDriver,
    IGPS *gpsDriver,
    IIMU *imuDriver,
    IMessageQueue<RCMotorControlMessage_t> *amQueue,
    IMessageQueue<TMMessage_t> *tmQueue,
    IMessageQueue<char[100]> *smLoggerQueue,
    MotorGroupInstance_t *rollMotors,
    MotorGroupInstance_t *pitchMotors,
    MotorGroupInstance_t *yawMotors,
    MotorGroupInstance_t *throttleMotors,
    MotorGroupInstance_t *flapMotors,
    MotorGroupInstance_t *steeringMotors
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
    rollMotors(rollMotors),
    pitchMotors(pitchMotors),
    yawMotors(yawMotors),
    throttleMotors(throttleMotors),
    flapMotors(flapMotors),
    steeringMotors(steeringMotors),
    lastServoOutputs{0},
    amSchedulingCounter(0),
    noDataCount(0),
    failsafeTriggered(false) {

    // Set PID constants and rudder mixing constant for FBWA control law
    fbwaCLAW.setRollPIDConstants(
        AM_FBWA_ROLL_P_GAIN,
        AM_FBWA_ROLL_I_GAIN,
        AM_FBWA_ROLL_D_GAIN,
        AM_FBWA_ROLL_D_TAU
    );
    fbwaCLAW.setPitchPIDConstants(
        AM_FBWA_PITCH_P_GAIN,
        AM_FBWA_PITCH_I_GAIN,
        AM_FBWA_PITCH_D_GAIN,
        AM_FBWA_PITCH_D_TAU
    );
    fbwaCLAW.setYawRudderMixingConstant(AM_FBWA_RUDDER_MIXING);

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
    RawImu_t imuData;
    ScaledImu_t scaledImuData;
    Attitude_t attitude;

    ZP_RETURN_IF_ERROR(imuDriver->readRawData(&imuData));
    ZP_RETURN_IF_ERROR(imuDriver->scaleIMUData(imuData, &scaledImuData));
    ZP_RETURN_IF_ERROR(mahonyFilter.updateIMU(
        scaledImuData->xgyro,
        scaledImuData->ygyro,
        scaledImuData->zgyro,
        scaledImuData->xacc,
        scaledImuData->yacc,
        scaledImuData->zacc
    ));
    ZP_RETURN_IF_ERROR(mahonyFilter.getAttitudeRadians(&attitude));
    droneState.roll = attitude.roll;
    droneState.pitch = attitude.pitch;
    droneState.yaw = attitude.yaw;

    if (amSchedulingCounter % (AM_SCHEDULING_RATE_HZ / AM_TELEMETRY_RAW_IMU_DATA_RATE_HZ) == 0) {
        ZP_RETURN_IF_ERROR(sendRawIMUDataToTelemetryManager(imuData));
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

        if (noDataCount * AM_UPDATE_LOOP_DELAY_MS > AM_FAILSAFE_TIMEOUT_MS) {
            outputToMotor(YAW, 50);
            outputToMotor(PITCH, 50);
            outputToMotor(ROLL, 50);
            outputToMotor(THROTTLE, 0);
            outputToMotor(FLAP_ANGLE, 0);
            outputToMotor(STEERING, 50);

            if (!failsafeTriggered) {
              char errorMsg[100] = "Failsafe triggered";
              ZP_RETURN_IF_ERROR(smLoggerQueue->push(&errorMsg));
              failsafeTriggered = true;
            }
        }
        return ZP_ERROR_OK;
    } else {
        noDataCount = 0;

        if (failsafeTriggered) {
          char errorMsg[100] = "Motor control restored";
          ZP_RETURN_IF_ERROR(smLoggerQueue->push(&errorMsg));
          failsafeTriggered = false;
        }
    }

    // Disarm
    if (controlMsg.arm == 0) {
        controlMsg.throttle = 0;
    }

    if (controlMsg.flightMode != currentFlightMode) {
        switch (controlMsg.flightMode) {
            case PlaneFlightMode_e::MANUAL:
                activeCLAW = &manualCLAW;
                break;
            case PlaneFlightMode_e::FBWA:
                activeCLAW = &fbwaCLAW;
                break;
        }
        ZP_RETURN_IF_ERROR(activeCLAW->activateFlightMode());
        currentFlightMode = controlMsg.flightMode;
    }

    RCMotorControlMessage_t motorOutputs = controlAlgorithm.runControl(controlMsg, droneState);

    ZP_RETURN_IF_ERROR(outputToMotor(YAW, motorOutputs.yaw));
    ZP_RETURN_IF_ERROR(outputToMotor(PITCH, motorOutputs.pitch));
    ZP_RETURN_IF_ERROR( outputToMotor(ROLL, motorOutputs.roll));
    ZP_RETURN_IF_ERROR(outputToMotor(THROTTLE, motorOutputs.throttle));
    ZP_RETURN_IF_ERROR(outputToMotor(FLAP_ANGLE, motorOutputs.flapAngle));
    ZP_RETURN_IF_ERROR(outputToMotor(STEERING, motorOutputs.yaw));

    if (controlMsg.flightMode != currentFlightMode) {
        switch (controlMsg.flightMode) {
            case PlaneFlightMode_e::MANUAL:
                activeCLAW = &manualCLAW;
                break;
            case PlaneFlightMode_e::FBWA:
                activeCLAW = &fbwaCLAW;
                break;
        }
        activeCLAW->activateFlightMode();
        currentFlightMode = controlMsg.flightMode;
    }

    RCMotorControlMessage_t motorOutputs = activeCLAW->runControl(controlMsg, droneState);

    outputToMotor(YAW, motorOutputs.yaw);
    outputToMotor(PITCH, motorOutputs.pitch);
    outputToMotor(ROLL, motorOutputs.roll);
    outputToMotor(THROTTLE, motorOutputs.throttle);
    outputToMotor(FLAP_ANGLE, motorOutputs.flapAngle);
    outputToMotor(STEERING, motorOutputs.yaw);
}

bool AttitudeManager::getControlInputs(RCMotorControlMessage_t *pControlMsg) {
    if (amQueue->count() == 0) {
        return false;
    }

    ZP_RETURN_IF_ERROR(amQueue->get(pControlMsg));

    return ZP_ERROR_OK;
}

ZP_ERROR_e AttitudeManager::outputToMotor(ControlAxis_t axis, uint8_t percent) {
    MotorGroupInstance_t *motorGroup = nullptr;

    switch (axis) {
        case ROLL:
            motorGroup = rollMotors;
            break;
        case PITCH:
            motorGroup = pitchMotors;
            break;
        case YAW:
            motorGroup = yawMotors;
            break;
        case THROTTLE:
            motorGroup = throttleMotors;
            break;
        case FLAP_ANGLE:
            motorGroup = flapMotors;
            break;
        case STEERING:
            motorGroup = steeringMotors;
            break;
        default:
            return ZP_ERROR_INVALID_PARAM;
    }

    for (uint8_t i = 0; i < motorGroup->motorCount; i++) {
        MotorInstance_t *motor = (motorGroup->motors + i);

        int32_t cmd = (int32_t)percent + motor->trim;

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
        uint8_t servoIdx = motor->motorInstance->getServoIdx();
        if (servoIdx < 16)
            lastServoOutputs[servoIdx - 1] = 1000 + (cmd * 10); // Convert to microseconds for telemetry

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

    ZP_RETURN_IF_ERROR(tmQueue->push(&gpsDataMsg));
    return ZP_ERROR_OK;
}

ZP_ERROR_e AttitudeManager::sendRawIMUDataToTelemetryManager(const RawImu_t &imuData) {
    TMMessage_t imuDataMsg;
    ZP_RETURN_IF_ERROR(rawImuDataPack(
        &imuDataMsg,
        systemUtilsDriver->getCurrentTimestampMs(), // time_boot_ms
        imuData.xacc,
        imuData.yacc,
        imuData.zacc,
        imuData.xgyro,
        imuData.ygyro,
        imuData.zgyro
    ));

    ZP_RETURN_IF_ERROR(tmQueue->push(&imuDataMsg));
    return ZP_ERROR_OK;
}

ZP_ERROR_e AttitudeManager::sendAttitudeDataToTelemetryManager(const Attitude_t &attitude) {
    TMMessage_t attitudeDataMsg;
    ZP_RETURN_IF_ERROR(attitudeDataPack(
        &attitudeDataMsg,
        systemUtilsDriver->getCurrentTimestampMs(), // time_boot_ms
        attitude.roll,
        attitude.pitch,
        attitude.yaw
    ));

    ZP_RETURN_IF_ERROR(tmQueue->push(&attitudeDataMsg));
    return ZP_ERROR_OK;
}

ZP_ERROR_e AttitudeManager::sendRawIMUDataToTelemetryManager(const RawImu_t &imuData) {
    TMMessage_t imuDataMsg;
    ZP_RETURN_IF_ERROR(rawImuDataPack(
        &imuDataMsg,
        systemUtilsDriver->getCurrentTimestampMs(), // time_boot_ms
        imuData.xacc,
        imuData.yacc,
        imuData.zacc,
        imuData.xgyro,
        imuData.ygyro,
        imuData.zgyro
    ));

    ZP_RETURN_IF_ERROR(tmQueue->push(&imuDataMsg));
    return ZP_ERROR_OK;
}

ZP_ERROR_e AttitudeManager::sendAttitudeDataToTelemetryManager(const Attitude_t &attitude) {
    TMMessage_t attitudeDataMsg;
    ZP_RETURN_IF_ERROR(attitudeDataPack(
        &attitudeDataMsg,
        systemUtilsDriver->getCurrentTimestampMs(), // time_boot_ms
        attitude.roll,
        attitude.pitch,
        attitude.yaw
    ));

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
