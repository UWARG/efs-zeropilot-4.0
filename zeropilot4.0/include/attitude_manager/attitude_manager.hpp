#pragma once

#include <cstdint>
#include "systemutils_iface.hpp"
#include "direct_mapping.hpp"
#include "fbwa_mapping.hpp"
#include "motor_datatype.hpp"
#include "gps_iface.hpp"
#include "tm_queue.hpp"
#include "imu_iface.hpp"
#include "ahrs_ekf.hpp"
#include "queue_iface.hpp"
#include "drone_state.hpp"
#include "am_param_setup.hpp"
#include "acro_mapping.hpp"
#include "stabilize_mapping.hpp"
#include "motor_mixing.hpp"
#include "fft_harmonic_notch.hpp"
#include "rangefinder_iface.hpp"
#include "barometer_iface.hpp"
#include "MahonyAHRS.hpp"
#include "althold_kf.hpp"

#define AM_SCHEDULING_RATE_HZ 1000
#define AM_TELEMETRY_GPS_DATA_RATE_HZ 5
#define AM_TELEMETRY_SCALED_PRESSURE_DATA_RATE_HZ 5
#define AM_TELEMETRY_RAW_IMU_DATA_RATE_HZ 10
#define AM_TELEMETRY_ATTITUDE_DATA_RATE_HZ 20
#define AM_TELEMETRY_SERVO_OUTPUT_RAW_RATE_HZ 2
#define AM_TELEMETRY_GLOBAL_POSITION_INT_RATE_HZ 10

#define AM_UPDATE_LOOP_DELAY_MS (1000 / AM_SCHEDULING_RATE_HZ)
#define AM_CONTROL_LOOP_PERIOD_S (static_cast<float>(AM_UPDATE_LOOP_DELAY_MS) / 1000.0f)

static_assert(AM_CONTROL_LOOP_PERIOD_S != 0.0f, "AM_CONTROL_LOOP_PERIOD_S must be nonzero.");

class AttitudeManager {
    friend class AMParamSetup;

public:
    AttitudeManager(
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
    );

    void amUpdate();

private:
    static constexpr uint8_t NUM_MOTORS = 8;

    ISystemUtils *systemUtilsDriver;

    IGPS *gpsDriver;
    IIMU *imuDriver;
    IRangefinder *rangefinderDriver;
    IBarometer *barometerDriver;

    FFTHarmonicNotch harmonicNotchFilter;
    FFTHarmonicNotchConfig harmonicNotchConfig;
    // AHRSEKF ekf;
    Mahony mahonyFilter;
    AltholdKF altholdKF;

    IMessageQueue<RCMotorControlMessage_t> *amQueue;
    IMessageQueue<TMMessage_t> *tmQueue;
    IMessageQueue<char[100]> *smLoggerQueue;

    Flightmode *activeCLAW; // Pointer to current active Control Law
#ifdef PLANE
    DirectMapping manualCLAW; // Manual Control Law (Direct Passthrough)
    FBWAMapping fbwaCLAW;     // Fly-By-Wire A Control Law (Roll and Pitch PID + Yaw Rudder Mixing)
#endif
#ifdef QUADCOPTER
    AcroMapping acroCLAW;           // Acro Control Law (Roll, Pitch and Yaw PID)
    StabilizeMapping stabilizeCLAW; // Stabilize Control Law (Roll, Pitch and Yaw PID + Angle Limiting)
#endif
    RCMotorControlMessage_t controlMsg;
    FlightMode_e currentFlightMode;
    MotorGroupInstance_t *mainMotorGroup;

    DroneState_t droneState = DRONE_STATE_DEFAULT;

    bool firstIteration = true;

    bool armedFlag = false;
    bool armStateChanged = false;

    uint16_t lastServoOutputs[16] = {};

    uint16_t amSchedulingCounter = 0;

    int noDataCount = 0;
    bool failsafeTriggered = false;

    float motSpinMin = 0.0f;
    float motSpinMax = 0.0f;
    float motSpinArm = 0.0f;

    static constexpr float MOT_GND_IDLE_THR = 0.02f;
    bool groundIdlePrev = false;

    static constexpr uint16_t MAX_TIMESTAMP = 65535;
    static constexpr float TIMESTAMP_RESOLUTION = 0.000001f; // Default IMU timestamp resolution 1us
    uint32_t lastTimestamp = 0;
    bool haveLastImuTimestamp = false;

    float altitude = 0.0f;

    float lastValidClearance = 0.0f;
    GpsData_t lastValidGps = {};
    bool gpsUnsent = false;

    // for debugging now
    BaroData_t baroData = {};
    RangefinderData_t rangefinderData = {};
    GpsData_t gpsData = {};

    float baroHomeAltitude = 0.0f;
    bool baroHomeInitialized = false;
    bool baroHomeSettled = false;

    float gpsHomeAltitude = 0.0f;
    bool gpsHomeInitialized = false;
    bool gpsHomeSettled = false;

    static constexpr float BARO_HOME_TIME_CONSTANT_S = 2.0f; // Tuned tradeoff: smooths baro sample noise but still tracks real drift over a long pre-arm wait
    static constexpr float BARO_UPDATE_DT_S = 1.0f / 25.0f; // Sample period is 25hz
    static constexpr float BARO_HOME_ALPHA = BARO_UPDATE_DT_S / (BARO_HOME_TIME_CONSTANT_S + BARO_UPDATE_DT_S);
    static constexpr float BARO_HOME_SETTLE_TIME_MS = 3 * BARO_HOME_TIME_CONSTANT_S * 1000.0f; // 3 time constants to settle the EMA (x1000 to convert to ms)
    uint32_t baroHomeCalibStartMs = 0;
    static constexpr float GPS_HOME_TIME_CONSTANT_S = 3.0f;
    static constexpr float GPS_UPDATE_DT_S = 1.0f / 5.0f; // 5hz
    static constexpr float GPS_HOME_ALPHA = GPS_UPDATE_DT_S / (GPS_HOME_TIME_CONSTANT_S + GPS_UPDATE_DT_S);
    static constexpr uint8_t  GPS_MIN_SATELLITES = 8;
    static constexpr float GPS_HOME_VACC_MAX_M = 3.0f;
    static constexpr float GPS_HOME_OUTLIER_M = 5.0f;
    static constexpr float GPS_HOME_STABLE_DELTA_M = 1.0f; 
    static constexpr uint32_t GPS_HOME_STABLE_TIME_MS = 10000;
    uint32_t gpsHomeCalibStartMs = 0; 

    float baroZ = 0.0f;
    float gpsZ = 0.0f;
    float rangefinderZ = 0.0f;

    // Motor mixer output for each motor
    float motorPercent[NUM_MOTORS] = {};
    
    uint8_t profilerId = 0;

    AMParamSetup paramSetup;

    bool getControlInputs(RCMotorControlMessage_t *pControlMsg);

    void outputToMotors(RCMotorControlMessage_t outputControlMsg, bool groundIdle);

    void sendGPSDataToTelemetryManager(const GpsData_t &gpsData);
    void sendRawIMUDataToTelemetryManager(const RawImu_t &imuData);
    void sendAttitudeDataToTelemetryManager(const Attitude_t &attitude);
    void sendPressureDataToTelemetryManager(const BaroData_t &baroData);
    void sendGlobalPositionIntToTelemetryManager(float altitudeAmsl, float altitudeRelative);
    void sendServoOutputRawToTelemetryManager();
};
