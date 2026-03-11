#include "attitude_manager.hpp"
#include "rc_motor_control.hpp"

#define AM_SCHEDULING_RATE_HZ 100
#define AM_TELEMETRY_GPS_DATA_RATE_HZ 5
#define AM_TELEMETRY_RAW_IMU_DATA_RATE_HZ 10
#define AM_TELEMETRY_ATTITUDE_DATA_RATE_HZ 20


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
    controlAlgorithm(),
    droneState(DRONE_STATE_DEFAULT),
    rollMotors(rollMotors),
    pitchMotors(pitchMotors),
    yawMotors(yawMotors),
    throttleMotors(throttleMotors),
    flapMotors(flapMotors),
    steeringMotors(steeringMotors),
    previouslyArmed(false),
    armAltitude(0.0f),
    amSchedulingCounter(0) {}

ZP_ERROR_e AttitudeManager::amUpdate() {

    int amSchedulingCounter = (amSchedulingCounter + 1) % AM_SCHEDULING_RATE_HZ;

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
        ZP_RETURN_IF_ERROR(sendAttitudeDataToTelemetryManager(attitude));
    }

    // Get data from Queue and motor outputs
    ZP_ERROR_e controlRes = getControlInputs(&controlMsg);

    // Failsafe
    static int noDataCount = 0;
    static bool failsafeTriggered = false;

    if (controlRes != ZP_ERROR_OK) {
        ++noDataCount;

        if (noDataCount * AM_MAIN_DELAY > 1000) {
            ZP_RETURN_IF_ERROR(outputToMotor(YAW, 50));
            ZP_RETURN_IF_ERROR(outputToMotor(PITCH, 50));
            ZP_RETURN_IF_ERROR(outputToMotor(ROLL, 50));
            ZP_RETURN_IF_ERROR(outputToMotor(THROTTLE, 0));
            ZP_RETURN_IF_ERROR(outputToMotor(FLAP_ANGLE, 0));
            ZP_RETURN_IF_ERROR(outputToMotor(STEERING, 50));
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

    // Send GPS data to telemetry manager
    GpsData_t gpsData;
    bool readStatus;
    ZP_RETURN_IF_ERROR(gpsDriver->readData(&gpsData, &readStatus));
    if (amSchedulingCounter % (AM_SCHEDULING_RATE_HZ / AM_TELEMETRY_GPS_DATA_RATE_HZ) == 0) {
        ZP_RETURN_IF_ERROR(sendGPSDataToTelemetryManager(gpsData, controlMsg.arm > 0));
    }


    RCMotorControlMessage_t motorOutputs;
    ZP_RETURN_IF_ERROR(controlAlgorithm.runControl(&motorOutputs, controlMsg, droneState));

    ZP_RETURN_IF_ERROR(outputToMotor(YAW, motorOutputs.yaw));
    ZP_RETURN_IF_ERROR(outputToMotor(PITCH, motorOutputs.pitch));
    ZP_RETURN_IF_ERROR( outputToMotor(ROLL, motorOutputs.roll));
    ZP_RETURN_IF_ERROR(outputToMotor(THROTTLE, motorOutputs.throttle));
    ZP_RETURN_IF_ERROR(outputToMotor(FLAP_ANGLE, motorOutputs.flapAngle));
    ZP_RETURN_IF_ERROR(outputToMotor(STEERING, motorOutputs.yaw));

    // Send GPS data to telemetry manager
    GpsData_t gpsData;
    ZP_RETURN_IF_ERROR(gpsDriver->readData(&gpsData));

    if (amSchedulingCounter % (AM_SCHEDULING_RATE_HZ / AM_TELEMETRY_GPS_DATA_RATE_HZ) == 0) {
        ZP_RETURN_IF_ERROR(sendGPSDataToTelemetryManager(gpsData, controlMsg.arm > 0));
    }

    amSchedulingCounter = (amSchedulingCounter + 1) % AM_SCHEDULING_RATE_HZ;

    return ZP_ERROR_OK;
}

ZP_ERROR_e AttitudeManager::getControlInputs(RCMotorControlMessage_t *pControlMsg) {
    int count = 0;
    ZP_RETURN_IF_ERROR(amQueue->count(&count));

    if (count == 0) {
        return ZP_ERROR_RESOURCE_UNAVAILABLE;
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

        if (motor->isInverted) {
            ZP_RETURN_IF_ERROR(motor->motorInstance->set(100 - percent));
        }
        else {
            ZP_RETURN_IF_ERROR(motor->motorInstance->set(percent));
        }
    }

    return ZP_ERROR_OK;
}


ZP_ERROR_e AttitudeManager::sendGPSDataToTelemetryManager(const GpsData_t &gpsData, const bool &armed) {
    if (!gpsData.isNew) return ZP_ERROR_OK;

    if (armed) {
        if (!previouslyArmed) {
            armAltitude = gpsData.altitude;
            previouslyArmed = true;
        }
    } else {
        previouslyArmed = false;
        armAltitude = 0.0f;
    }

    // calculate relative altitude
    float relativeAltitude = previouslyArmed ? (gpsData.altitude - armAltitude) : 0.0f;

    uint32_t timestampMs = 0;
    ZP_RETURN_IF_ERROR(systemUtilsDriver->getCurrentTimestampMs(&timestampMs));

    TMMessage_t gpsDataMsg;
    ZP_RETURN_IF_ERROR(gposDataPack(
        &gpsDataMsg,
        timestampMs, // time_boot_ms
        gpsData.altitude * 1000, // altitude in mm
        gpsData.latitude * 1e7,
        gpsData.longitude * 1e7,
        relativeAltitude * 1000, // relative altitude in mm
        gpsData.vx,
        gpsData.vy,
        gpsData.vz,
        gpsData.trackAngle
    ));

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

    ZP_RETURN_IF_ERROR(tmQueue->push(&attitudeDataMsg));
    return ZP_ERROR_OK;
}
