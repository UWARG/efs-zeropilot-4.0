#pragma once

#include <cstdint>
#include "systemutils_iface.hpp"
#include "direct_mapping.hpp"
#include "fbwa_mapping.hpp"
#include "motor_datatype.hpp"
#include "gps_iface.hpp"
#include "tm_queue.hpp"
#include "imu_iface.hpp"
#include "MahonyAHRS.hpp"
#include "queue_iface.hpp"
#include "drone_state.hpp"
#include "am_param_setup.hpp"
#include "acro_mapping.hpp"
#include "motor_mixing.hpp"
#include "stabilize_mapping.hpp"

#define AM_SCHEDULING_RATE_HZ 1000
#define AM_TELEMETRY_GPS_DATA_RATE_HZ 5
#define AM_TELEMETRY_RAW_IMU_DATA_RATE_HZ 10
#define AM_TELEMETRY_ATTITUDE_DATA_RATE_HZ 20
#define AM_TELEMETRY_SERVO_OUTPUT_RAW_RATE_HZ 2

#define AM_UPDATE_LOOP_DELAY_MS (1000 / AM_SCHEDULING_RATE_HZ)
#define AM_CONTROL_LOOP_PERIOD_S (static_cast<float>(AM_UPDATE_LOOP_DELAY_MS) / 1000.0f)

#ifdef QUADCOPTER
#define  STABILIZE_CONTROL_LOOP_PERIOD_S (static_cast<float>(AM_UPDATE_LOOP_DELAY_MS) / 100.0f)
#endif

class AttitudeManager {
    friend class AMParamSetup;

    public:
        AttitudeManager(
            ISystemUtils *systemUtilsDriver,
            IGPS *gpsDriver,
            IIMU *imuDriver,
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

        Mahony mahonyFilter;

        IMessageQueue<RCMotorControlMessage_t> *amQueue;
        IMessageQueue<TMMessage_t> *tmQueue;
        IMessageQueue<char[100]> *smLoggerQueue;

        Flightmode *activeCLAW;     // Pointer to current active Control Law
        #ifdef PLANE
        DirectMapping manualCLAW;   // Manual Control Law (Direct Passthrough)
        FBWAMapping fbwaCLAW;       // Fly-By-Wire A Control Law (Roll and Pitch PID + Yaw Rudder Mixing)
        #endif
        #ifdef QUADCOPTER
        ACROMapping acroCLAW;
        STABILIZEMapping stabilizeCLAW;
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

        AMParamSetup paramSetup;
};
