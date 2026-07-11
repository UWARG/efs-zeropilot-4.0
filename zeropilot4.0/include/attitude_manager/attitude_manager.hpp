#pragma once

#include <cstdint>
#include "systemutils_iface.hpp"
#include "direct_mapping.hpp"
#include "fbwa_mapping.hpp"
#include "fbwb_mapping.hpp"
#include "motor_datatype.hpp"
#include "gps_iface.hpp"
#include "airspeed_iface.hpp"
#include "tm_queue.hpp"
#include "imu_iface.hpp"
#include "MahonyAHRS.hpp"
#include "queue_iface.hpp"
#include "drone_state.hpp"
#include "am_param_setup.hpp"
#include "acro_mapping.hpp"
#include "stabilize_mapping.hpp"
#include "motor_mixing.hpp"
#include "fft_harmonic_notch.hpp"

#define AM_SCHEDULING_RATE_HZ 1000
#define AM_TELEMETRY_GPS_DATA_RATE_HZ 5
#define AM_TELEMETRY_RAW_IMU_DATA_RATE_HZ 10
#define AM_TELEMETRY_ATTITUDE_DATA_RATE_HZ 20
#define AM_TELEMETRY_SERVO_OUTPUT_RAW_RATE_HZ 2

#define AM_UPDATE_LOOP_DELAY_MS (1000 / AM_SCHEDULING_RATE_HZ)
#define AM_CONTROL_LOOP_PERIOD_S (static_cast<float>(AM_UPDATE_LOOP_DELAY_MS) / 1000.0f)

// PID constants for FBWB control law
static constexpr float AM_FBWB_TOTAL_ENERGY_P_GAIN = 0.0f; // TODO: set TE P-gain
static constexpr float AM_FBWB_TOTAL_ENERGY_I_GAIN = 0.0f; // TODO set TE I-gain
static constexpr float AM_FBWB_TOTAL_ENERGY_D_GAIN = 0.0f;
static constexpr float AM_FBWB_TOTAL_ENERGY_D_TAU = 0.02f;
static constexpr float AM_FBWB_ENERGY_BALANCE_P_GAIN = 0.0f; // TODO: set EB P-gain
static constexpr float AM_FBWB_ENERGY_BALANCE_I_GAIN = 0.0f; // TODO: set EB I-gain
static constexpr float AM_FBWB_ENERGY_BALANCE_D_GAIN = 0.0f;
static constexpr float AM_FBWB_ENERGY_BALANCE_D_TAU = 0.02f;

typedef enum {
    YAW = 0,
    PITCH,
    ROLL,
    THROTTLE,
    FLAP_ANGLE,
    STEERING
} ControlAxis_t;

class AttitudeManager {
    friend class AMParamSetup;

    public:
        AttitudeManager(
            ISystemUtils *systemUtilsDriver,
            IGPS *gpsDriver,
            IIMU *imuDriver,
            IFFT *fftDriver,
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
        IAirspeed *airspeedDriver;

        FFTHarmonicNotch harmonicNotchFilter;
        FFTHarmonicNotchConfig harmonicNotchConfig;
        Mahony mahonyFilter;

        IMessageQueue<RCMotorControlMessage_t> *amQueue;
        IMessageQueue<TMMessage_t> *tmQueue;
        IMessageQueue<char[100]> *smLoggerQueue;

        Flightmode *activeCLAW;     // Pointer to current active Control Law
        #ifdef PLANE
        DirectMapping manualCLAW;   // Manual Control Law (Direct Passthrough)
        FBWAMapping fbwaCLAW;       // Fly-By-Wire A Control Law (Roll and Pitch PID + Yaw Rudder Mixing)
        FBWBMapping fbwbCLAW;       // Fly-By-Wire B Control Law (TECS)
        #endif
        #ifdef QUADCOPTER
        AcroMapping acroCLAW;           // Acro Control Law (Roll, Pitch and Yaw PID)
        StabilizeMapping stabilizeCLAW; // Stabilize Control Law (Roll, Pitch and Yaw PID + Angle Limiting)
        #endif
        RCMotorControlMessage_t controlMsg;
        FlightMode_e currentFlightMode;
        DroneState_t droneState;

        MotorGroupInstance_t *mainMotorGroup;

        bool armedFlag;
        bool setArmFlag;

        uint16_t lastServoOutputs[16];

        uint16_t amSchedulingCounter;

        int noDataCount;
        bool failsafeTriggered;

        static constexpr uint16_t MAX_TIMESTAMP = 65535;
        static constexpr float TIMESTAMP_RESOLUTION = 0.000001f; // Default IMU timestamp resolution 1us
        uint32_t lastTimestamp;
        bool haveLastImuTimestamp;

        bool getControlInputs(RCMotorControlMessage_t *pControlMsg);

        void outputToMotors(RCMotorControlMessage_t outputControlMsg);

        void sendGPSDataToTelemetryManager(const GpsData_t &gpsData);
        void sendRawIMUDataToTelemetryManager(const RawImu_t &imuData);
        void sendAttitudeDataToTelemetryManager(const Attitude_t &attitude);
        void sendServoOutputRawToTelemetryManager();
        
        uint8_t profilerId;
        
        // Motor mixer output for each motor 
        float motorPercent[NUM_MOTORS];

        // ZP_PARAM callback functions
        static bool updatePIDRollKp(AttitudeManager* context, float val);
        static bool updatePIDRollKi(AttitudeManager* context, float val);
        static bool updatePIDRollKd(AttitudeManager* context, float val);
        static bool updatePIDRollTau(AttitudeManager* context, float val);
        static bool updatePIDRollIMax(AttitudeManager* context, float val);
        static bool updatePIDPitchKp(AttitudeManager* context, float val);
        static bool updatePIDPitchKi(AttitudeManager* context, float val);
        static bool updatePIDPitchKd(AttitudeManager* context, float val);
        static bool updatePIDPitchTau(AttitudeManager* context, float val);
        static bool updatePIDPitchIMax(AttitudeManager* context, float val);
        static bool updateKffRddrmix(AttitudeManager* context, float val);
        static bool updateRollLimitDeg(AttitudeManager* context, float val);
        static bool updatePitchLimMaxDeg(AttitudeManager* context, float val);
        static bool updatePitchLimMinDeg(AttitudeManager* context, float val);
        static bool updateFBWBTEKp(AttitudeManager* context, float val);
        static bool updateFBWBTEKi(AttitudeManager* context, float val);
        static bool updateFBWBTEKd(AttitudeManager* context, float val);
        static bool updateFBWBTETau(AttitudeManager* context, float val);
        static bool updateFBWBEBKp(AttitudeManager* context, float val);
        static bool updateFBWBEBKi(AttitudeManager* context, float val);
        static bool updateFBWBEBKd(AttitudeManager* context, float val);
        static bool updateFBWBEBTau(AttitudeManager* context, float val);

        AMParamSetup paramSetup;
};
