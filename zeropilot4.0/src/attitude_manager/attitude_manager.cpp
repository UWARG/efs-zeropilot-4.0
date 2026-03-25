#include "attitude_manager.hpp"
#include "rc_motor_control.hpp"
#include "zp_params.hpp"

AttitudeManager::AttitudeManager(
    ISystemUtils *systemUtilsDriver,
    IGPS *gpsDriver,
    IIMU *imuDriver,
    IAirspeed *airspeedDriver,
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
    airspeedDriver(airspeedDriver),
    amQueue(amQueue),
    tmQueue(tmQueue),
    smLoggerQueue(smLoggerQueue),
    activeCLAW(&manualCLAW),
    manualCLAW(),
    fbwaCLAW(AM_CONTROL_LOOP_PERIOD_S),
    fbwbCLAW(AM_CONTROL_LOOP_PERIOD_S),
    controlMsg({50, 50, 50, 0, 0, 0, PlaneFlightMode_e::MANUAL}),
    droneState(DRONE_STATE_DEFAULT),
    currentFlightMode(PlaneFlightMode_e::MANUAL),
    rollMotors(rollMotors),
    pitchMotors(pitchMotors),
    yawMotors(yawMotors),
    throttleMotors(throttleMotors),
    flapMotors(flapMotors),
    steeringMotors(steeringMotors),
    armedFlag(false),
    lastServoOutputs{0},
    amSchedulingCounter(0),
    noDataCount(0),
    failsafeTriggered(false) {

    // Bind all ZP Param setters relevant to AM
    ZP_PARAM::bindCallback(ZP_PARAM_ID::RLL2SRV_P, this, AttitudeManager::updatePIDRollKp);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::RLL2SRV_I, this, AttitudeManager::updatePIDRollKi);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::RLL2SRV_D, this, AttitudeManager::updatePIDRollKd);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::RLL2SRV_TAU, this, AttitudeManager::updatePIDRollTau);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::RLL2SRV_IMAX, this, AttitudeManager::updatePIDRollIMax);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::PTCH2SRV_P, this, AttitudeManager::updatePIDPitchKp);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::PTCH2SRV_I, this, AttitudeManager::updatePIDPitchKi);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::PTCH2SRV_D, this, AttitudeManager::updatePIDPitchKd);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::PTCH2SRV_TAU, this, AttitudeManager::updatePIDPitchTau);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::PTCH2SRV_IMAX, this, AttitudeManager::updatePIDPitchIMax);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::KFF_RDDRMIX, this, AttitudeManager::updateKffRddrmix);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::ROLL_LIMIT_DEG, this, AttitudeManager::updateRollLimitDeg);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::PTCH_LIM_MAX_DEG, this, AttitudeManager::updatePitchLimMaxDeg);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::PTCH_LIM_MIN_DEG, this, AttitudeManager::updatePitchLimMinDeg);

    ZP_PARAM::bindCallback(ZP_PARAM_ID::AM_FBWB_TOTAL_ENERGY_P_GAIN, this, AttitudeManager::updateFBWBTEKp);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::AM_FBWB_TOTAL_ENERGY_I_GAIN, this, AttitudeManager::updateFBWBTEKi);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::AM_FBWB_TOTAL_ENERGY_D_GAIN, this, AttitudeManager::updateFBWBTEKd);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::AM_FBWB_TOTAL_ENERGY_D_TAU, this, AttitudeManager::updateFBWBTETau);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::AM_FBWB_ENERGY_BALANCE_P_GAIN, this, AttitudeManager::updateFBWBEBKp);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::AM_FBWB_ENERGY_BALANCE_I_GAIN, this, AttitudeManager::updateFBWBEBKi);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::AM_FBWB_ENERGY_BALANCE_D_GAIN, this, AttitudeManager::updateFBWBEBKd);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::AM_FBWB_ENERGY_BALANCE_D_TAU, this, AttitudeManager::updateFBWBEBTau);

    // Set PID constants, rddr mixing constant, and roll/pitch limits for FBWA control law
    fbwaCLAW.setRollPIDConstants(
        ZP_PARAM::get(ZP_PARAM_ID::RLL2SRV_P),
        ZP_PARAM::get(ZP_PARAM_ID::RLL2SRV_I),
        ZP_PARAM::get(ZP_PARAM_ID::RLL2SRV_D),
        ZP_PARAM::get(ZP_PARAM_ID::RLL2SRV_TAU),
        ZP_PARAM::get(ZP_PARAM_ID::RLL2SRV_IMAX)
    );
    fbwaCLAW.setPitchPIDConstants(
        ZP_PARAM::get(ZP_PARAM_ID::PTCH2SRV_P),
        ZP_PARAM::get(ZP_PARAM_ID::PTCH2SRV_I),
        ZP_PARAM::get(ZP_PARAM_ID::PTCH2SRV_D),
        ZP_PARAM::get(ZP_PARAM_ID::PTCH2SRV_TAU),
        ZP_PARAM::get(ZP_PARAM_ID::PTCH2SRV_IMAX)
    );
    fbwaCLAW.setYawRudderMixingConstant(ZP_PARAM::get(ZP_PARAM_ID::KFF_RDDRMIX));
    fbwaCLAW.setRollLimitDeg(ZP_PARAM::get(ZP_PARAM_ID::ROLL_LIMIT_DEG));
    fbwaCLAW.setPitchLimitMaxDeg(ZP_PARAM::get(ZP_PARAM_ID::PTCH_LIM_MAX_DEG));
    fbwaCLAW.setPitchLimitMinDeg(ZP_PARAM::get(ZP_PARAM_ID::PTCH_LIM_MIN_DEG));

    // TODO: Load in FBWB PID control constants here

    // Set PID constants for FBWB control law
    fbwbCLAW.setRollPIDConstants(
        ZP_PARAM::get(ZP_PARAM_ID::RLL2SRV_P),
        ZP_PARAM::get(ZP_PARAM_ID::RLL2SRV_I),
        ZP_PARAM::get(ZP_PARAM_ID::RLL2SRV_D),
        ZP_PARAM::get(ZP_PARAM_ID::RLL2SRV_TAU),
        ZP_PARAM::get(ZP_PARAM_ID::RLL2SRV_IMAX)
    );
    fbwbCLAW.setPitchPIDConstants(
        ZP_PARAM::get(ZP_PARAM_ID::PTCH2SRV_P),
        ZP_PARAM::get(ZP_PARAM_ID::PTCH2SRV_I),
        ZP_PARAM::get(ZP_PARAM_ID::PTCH2SRV_D),
        ZP_PARAM::get(ZP_PARAM_ID::PTCH2SRV_TAU),
        ZP_PARAM::get(ZP_PARAM_ID::PTCH2SRV_IMAX)
    );
    fbwbCLAW.setEnergyBalancePIDConstants(
        ZP_PARAM::get(ZP_PARAM_ID::AM_FBWB_ENERGY_BALANCE_P_GAIN),
        ZP_PARAM::get(ZP_PARAM_ID::AM_FBWB_ENERGY_BALANCE_I_GAIN),
        ZP_PARAM::get(ZP_PARAM_ID::AM_FBWB_ENERGY_BALANCE_D_GAIN),
        ZP_PARAM::get(ZP_PARAM_ID::AM_FBWB_ENERGY_BALANCE_D_TAU),
        100 //Learn energybalance max value
    );
    fbwbCLAW.setTotalEnergyPIDConstants(
        ZP_PARAM::get(ZP_PARAM_ID::AM_FBWB_TOTAL_ENERGY_P_GAIN),
        ZP_PARAM::get(ZP_PARAM_ID::AM_FBWB_TOTAL_ENERGY_I_GAIN),
        ZP_PARAM::get(ZP_PARAM_ID::AM_FBWB_TOTAL_ENERGY_D_GAIN),
        ZP_PARAM::get(ZP_PARAM_ID::AM_FBWB_TOTAL_ENERGY_D_TAU),
        100
    );
    fbwbCLAW.setYawRudderMixingConstant(ZP_PARAM::get(ZP_PARAM_ID::KFF_RDDRMIX));

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
    droneState.altitude = gpsData.altitude;
    if (amSchedulingCounter % (AM_SCHEDULING_RATE_HZ / AM_TELEMETRY_GPS_DATA_RATE_HZ) == 0) {
        sendGPSDataToTelemetryManager(gpsData);
    }

    // Read airspeed
    double airspeedData = 0.0f;
    if (airspeedDriver->getAirspeedData(&airspeedData)) {
        droneState.airspeed = airspeedData;
    }

    // Get data from Queue and motor outputs
    bool controlRes = getControlInputs(&controlMsg);
    
    if (controlRes != true) {
        ++noDataCount;

        if (noDataCount * AM_UPDATE_LOOP_DELAY_MS > ((ZP_PARAM::get(ZP_PARAM_ID::RC_FS_TIMEOUT)) * 1000)) {
            outputToMotor(YAW, 50);
            outputToMotor(PITCH, 50);
            outputToMotor(ROLL, 50);
            outputToMotor(THROTTLE, 0);
            outputToMotor(FLAP_ANGLE, 0);
            outputToMotor(STEERING, 50);

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
        armedFlag = controlMsg.arm;
        if (armedFlag) {
            activeCLAW->activateFlightMode();
        }
    }

    // Update current flightmode if changed
    if (controlMsg.flightMode != currentFlightMode) {
        switch (controlMsg.flightMode) {
            case PlaneFlightMode_e::MANUAL:
                activeCLAW = &manualCLAW;
                break;
            case PlaneFlightMode_e::FBWA:
                activeCLAW = &fbwaCLAW;
                break;
            case PlaneFlightMode_e::FBWB:
                activeCLAW = &fbwbCLAW;
                break;
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

    // Output to motors
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

    amQueue->get(pControlMsg);
    return true;
}

void AttitudeManager::outputToMotor(ControlAxis_t axis, uint8_t percent) {
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
            return;
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

// STATIC FUNCTIONS ONLY FOR PARAM CHAINING
// ==============================================================


bool AttitudeManager::updatePIDRollKp(AttitudeManager* context, float val) {
    if (val < 0.0f) return false;

    context->fbwaCLAW.getRollPID()->setKp(val);
    return true;
}

bool AttitudeManager::updatePIDRollKi(AttitudeManager* context, float val) {
    if (val < 0.0f) return false;

    context->fbwaCLAW.getRollPID()->setKi(val);
    return true;
}

bool AttitudeManager::updatePIDRollKd(AttitudeManager* context, float val) {
    if (val < 0.0f) return false;

    context->fbwaCLAW.getRollPID()->setKd(val);
    return true;
}

bool AttitudeManager::updatePIDRollTau(AttitudeManager* context, float val) {
    if (val < 0.0f) return false;

    context->fbwaCLAW.getRollPID()->setTau(val);
    return true;
}

bool AttitudeManager::updatePIDRollIMax(AttitudeManager* context, float val) {
    if (val < 0.0f || val > 100.0f) return false;

    context->fbwaCLAW.getRollPID()->setIntegralMinLimPct(static_cast<uint8_t>(val));
    context->fbwaCLAW.getRollPID()->setIntegralMaxLimPct(static_cast<uint8_t>(val));
    return true;
}

bool AttitudeManager::updatePIDPitchKp(AttitudeManager* context, float val) {
    if (val < 0.0f) return false;

    context->fbwaCLAW.getPitchPID()->setKp(val);
    return true;
}

bool AttitudeManager::updatePIDPitchKi(AttitudeManager* context, float val) {
    if (val < 0.0f) return false;

    context->fbwaCLAW.getPitchPID()->setKi(val);
    return true;
}

bool AttitudeManager::updatePIDPitchKd(AttitudeManager* context, float val) {
    if (val < 0.0f) return false;

    context->fbwaCLAW.getPitchPID()->setKd(val);
    return true;
}

bool AttitudeManager::updatePIDPitchTau(AttitudeManager* context, float val) {
    if (val < 0.0f) return false;

    context->fbwaCLAW.getPitchPID()->setTau(val);
    return true;
}

bool AttitudeManager::updatePIDPitchIMax(AttitudeManager* context, float val) {
    if (val < 0.0f || val > 100.0f) return false;

    context->fbwaCLAW.getPitchPID()->setIntegralMinLimPct(static_cast<uint8_t>(val));
    context->fbwaCLAW.getPitchPID()->setIntegralMaxLimPct(static_cast<uint8_t>(val));
    return true;
}

bool AttitudeManager::updateKffRddrmix(AttitudeManager* context, float val) {
    if (val < 0.0f || val > 1.0f) return false;

    context->fbwaCLAW.setYawRudderMixingConstant(val);
    return true;
}

bool AttitudeManager::updateRollLimitDeg(AttitudeManager* context, float val) {
    if (val < 0.0f || val > 90.0f) return false;

    context->fbwaCLAW.setRollLimitDeg(val);
    return true;
}

bool AttitudeManager::updatePitchLimMaxDeg(AttitudeManager* context, float val) {
    if (val < 0.0f || val > 90.0f) return false;
    
    context->fbwaCLAW.setPitchLimitMaxDeg(val);
    return true;
}

bool AttitudeManager::updatePitchLimMinDeg(AttitudeManager* context, float val) {
    if (val < -90.0f || val > 0.0f) return false;
    
    context->fbwaCLAW.setPitchLimitMinDeg(val);
    return true;
}

//TODO: Finish parameterization
//TODO: learn energy balance max value for I term
//CONFIRM: why attitude manager has access to FBWB PID constants but not FBWA PID constants? should we be able to change FBWA PID consts in flight as well?
//CONFIRM: zp_params.cpp initial values
//Next steps: PieceWise Scaling mapping.cpp
//Next steps: Slew rate

bool AttitudeManager::updateFBWBTEKp(AttitudeManager* context, float val) {
    if (val < 0.0f) return false;
    
    context->fbwbCLAW.getTotalEnergyPID()->setKp(val); 
    return true; 
}

bool AttitudeManager::updateFBWBTEKi(AttitudeManager* context, float val) {
    if (val < 0.0f) return false;
    
    context->fbwbCLAW.getTotalEnergyPID()->setKi(val); 
    return true; 
}

bool AttitudeManager::updateFBWBTEKd(AttitudeManager* context, float val) {
    if (val < 0.0f) return false;
    
    context->fbwbCLAW.getTotalEnergyPID()->setKd(val); 
    return true;
}

bool AttitudeManager::updateFBWBTETau(AttitudeManager* context, float val) {
    if (val < 0.0f) return false;
    
    context->fbwbCLAW.getTotalEnergyPID()->setTau(val); 
    return true; 
}

bool AttitudeManager::updateFBWBEBKp(AttitudeManager* context, float val) {
    if (val < 0.0f) return false;
    
    context->fbwbCLAW.getEnergyBalancePID()->setKp(val);
    return true;
}

bool AttitudeManager::updateFBWBEBKi(AttitudeManager* context, float val) {
    if (val < 0.0f) return false;
    
    context->fbwbCLAW.getEnergyBalancePID()->setKi(val);
    return true;
}

bool AttitudeManager::updateFBWBEBKd(AttitudeManager* context, float val) {
    if (val < 0.0f) return false;
    
    context->fbwbCLAW.getEnergyBalancePID()->setKd(val);
    return true;
}

bool AttitudeManager::updateFBWBEBTau(AttitudeManager* context, float val) {
    if (val < 0.0f) return false;
    
    context->fbwbCLAW.getEnergyBalancePID()->setTau(val);
    return true;
}
// ==============================================================
