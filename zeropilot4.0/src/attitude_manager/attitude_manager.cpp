#include "attitude_manager.hpp"
#include "rc_motor_control.hpp"
#include "zp_params.hpp"
#include "motor_functions.hpp"

// Number of servo param fields per servo channel
static constexpr uint8_t SERVO_PARAMS_PER_CHANNEL = 5;

// Convert PWM microseconds (1000-2000) to percent (0-100)
static inline int usToPercent(float us) {
    return static_cast<int>((us - 1000.0f) / 10.0f);
}

AttitudeManager::AttitudeManager(
    ISystemUtils *systemUtilsDriver,
    IGPS *gpsDriver,
    IIMU *imuDriver,
    IBarometer *barometerDriver,
    IMessageQueue<RCMotorControlMessage_t> *amQueue,
    IMessageQueue<TMMessage_t> *tmQueue,
    IMessageQueue<char[100]> *smLoggerQueue,
    MotorGroupInstance_t *mainMotorGroup
) :
    systemUtilsDriver(systemUtilsDriver),
    gpsDriver(gpsDriver),
    imuDriver(imuDriver),
    barometerDriver(barometerDriver),
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

    // Activate the activeCLAW
    activeCLAW->activateFlightMode();

    // Load servo parameters from ZP_PARAM into motor instances and bind callbacks
    loadServoParams();
    bindServoParamCallbacks();
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
    outputToMotors(motorOutputs);
}

bool AttitudeManager::getControlInputs(RCMotorControlMessage_t *pControlMsg) {
    if (amQueue->count() == 0) {
        return false;
    }

    amQueue->get(pControlMsg);
    return true;
}

void AttitudeManager::outputToMotors(RCMotorControlMessage_t outputControlMsg) {
    for (uint8_t i = 0; i < mainMotorGroup->motorCount; i++) {
        // Get current motor
        MotorInstance_t *motor = (mainMotorGroup->motors + i);

        // Extract percentage based on function
        float percent = 0.0f;
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

        // Set cmd based on percent and trim, min, max
        uint32_t cmd = 0;
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

        // Store for telemetry output
        lastServoOutputs[i] = 1000 + (cmd * 10); // Convert to microseconds for telemetry

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

// SERVO PARAM LOADING AND CALLBACKS
// ==============================================================
// ---------------------------------------------------------------------------
// loadServoParams: reads servo config from ZP_PARAM into motor instances
//
// The ZP_PARAM_ID enum lays out servo params in groups of 5 (SERVO_PARAMS_PER_CHANNEL):
//   SERVO1_TRIM, SERVO1_MIN, SERVO1_MAX, SERVO1_REVERSED, SERVO1_FUNCTION,
//   SERVO2_TRIM, SERVO2_MIN, ...
// So for channel i, the base param ID is SERVO1_TRIM + i*5, and offsets +0..+4
// give TRIM, MIN, MAX, REVERSED, FUNCTION respectively.
// ---------------------------------------------------------------------------
void AttitudeManager::loadServoParams() {
    for (uint8_t i = 0; i < mainMotorGroup->motorCount; i++) {
        ZP_PARAM_ID base = static_cast<ZP_PARAM_ID>(
            static_cast<uint16_t>(ZP_PARAM_ID::SERVO1_TRIM) + i * SERVO_PARAMS_PER_CHANNEL);
        MotorInstance_t *m = &mainMotorGroup->motors[i];
        m->trim      = usToPercent(ZP_PARAM::get(base));                                                                    // base + 0
        m->min       = usToPercent(ZP_PARAM::get(static_cast<ZP_PARAM_ID>(static_cast<uint16_t>(base) + 1)));               // base + 1
        m->max       = usToPercent(ZP_PARAM::get(static_cast<ZP_PARAM_ID>(static_cast<uint16_t>(base) + 2)));               // base + 2
        m->isInverted = static_cast<int>(ZP_PARAM::get(static_cast<ZP_PARAM_ID>(static_cast<uint16_t>(base) + 3))) != 0;    // base + 3
        m->function  = static_cast<MotorFunction_e>(static_cast<int16_t>(ZP_PARAM::get(static_cast<ZP_PARAM_ID>(static_cast<uint16_t>(base) + 4))));  // base + 4
    }
}

// ---------------------------------------------------------------------------
// updateServoParam<Idx>: compile-time generated callback for each servo param
//
// The template parameter Idx is a flat index into the servo parameter table:
//   Idx 0 = SERVO1_TRIM, 1 = SERVO1_MIN, 2 = SERVO1_MAX, 3 = SERVO1_REVERSED,
//   4 = SERVO1_FUNCTION, 5 = SERVO2_TRIM, 6 = SERVO2_MIN, ... and so on.
//
// At compile time, Idx is divided by SERVO_PARAMS_PER_CHANNEL (5) to get the
// servo channel number, and the remainder gives the field within that channel.
// Because Idx is a compile-time constant (template parameter), the compiler
// optimizes each instantiation into a direct write to the correct motor/field
// with no runtime lookup overhead.
//
// The compiler generates one distinct function for every Idx value (0..59 for
// 12 channels × 5 params), each of which is registered as the ZP_PARAM callback
// for the corresponding parameter ID.
// ---------------------------------------------------------------------------
template <uint8_t Idx>
bool AttitudeManager::updateServoParam(AttitudeManager* ctx, float val) {
    constexpr uint8_t CHANNEL = Idx / SERVO_PARAMS_PER_CHANNEL;
    constexpr uint8_t FIELD   = Idx % SERVO_PARAMS_PER_CHANNEL;
    if (CHANNEL >= ctx->mainMotorGroup->motorCount) return false;

    MotorInstance_t *m = &ctx->mainMotorGroup->motors[CHANNEL];
    switch (FIELD) {
        case 0: m->trim      = usToPercent(val); break;
        case 1: m->min       = usToPercent(val); break;
        case 2: m->max       = usToPercent(val); break;
        case 3: m->isInverted = static_cast<int>(val) != 0; break;
        case 4: m->function  = static_cast<MotorFunction_e>(static_cast<int16_t>(val)); break;
    }
    return true;
}

// ---------------------------------------------------------------------------
// bindServoParamCallbacksImpl: registers all servo param callbacks at once
//
// Uses C++14 std::integer_sequence + parameter pack expansion to avoid writing
// 60 individual bindCallback() calls by hand.
//
// How it works step by step:
//   1. The caller passes std::make_integer_sequence<uint8_t, 60>{}, which is a
//      type that carries the compile-time list <0, 1, 2, ... 59>.
//   2. The template parameter pack "Is..." captures that list.
//   3. The "expand" trick creates a dummy int array whose initializer list
//      contains one bindCallback() call per value in Is...:
//        {0, (bindCallback(..., updateServoParam<0>), 0),
//             (bindCallback(..., updateServoParam<1>), 0),
//             ...
//             (bindCallback(..., updateServoParam<59>), 0)}
//      The comma operator (expr, 0) discards bindCallback's return value and
//      yields 0 for each array element. The leading 0 handles the empty-pack
//      edge case (zero-length arrays are illegal in C++).
//   4. (void) suppresses the unused-variable warning on the dummy array.
//
// Net effect: one line of template code expands to 60 bindCallback() calls
// at compile time, one for each servo parameter.
// ---------------------------------------------------------------------------
template <uint8_t... Is>
void AttitudeManager::bindServoParamCallbacksImpl(std::integer_sequence<uint8_t, Is...>) {
    using expand = int[];
    (void)expand{0, (ZP_PARAM::bindCallback(
        static_cast<ZP_PARAM_ID>(static_cast<uint16_t>(ZP_PARAM_ID::SERVO1_TRIM) + Is),
        this, AttitudeManager::updateServoParam<Is>), 0)...};
}

// Total number of servo parameters: 12 channels × 5 params each = 60
static constexpr uint8_t TOTAL_SERVO_PARAMS = 12 * SERVO_PARAMS_PER_CHANNEL;

void AttitudeManager::bindServoParamCallbacks() {
    // std::make_integer_sequence<uint8_t, 60> generates the type
    // std::integer_sequence<uint8_t, 0, 1, 2, ..., 59> which is passed to
    // bindServoParamCallbacksImpl to expand into 60 bindCallback() calls.
    bindServoParamCallbacksImpl(std::make_integer_sequence<uint8_t, TOTAL_SERVO_PARAMS>{});
}
// ==============================================================

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
// ==============================================================
