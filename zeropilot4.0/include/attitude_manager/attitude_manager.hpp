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

#define AM_SCHEDULING_RATE_HZ 100
#define AM_TELEMETRY_GPS_DATA_RATE_HZ 5
#define AM_TELEMETRY_RAW_IMU_DATA_RATE_HZ 10
#define AM_TELEMETRY_ATTITUDE_DATA_RATE_HZ 20
#define AM_TELEMETRY_SERVO_OUTPUT_RAW_RATE_HZ 2

#define AM_UPDATE_LOOP_DELAY_MS (1000 / AM_SCHEDULING_RATE_HZ)
#define AM_CONTROL_LOOP_PERIOD_S (static_cast<float>(AM_UPDATE_LOOP_DELAY_MS) / 1000.0f)
#define AM_FAILSAFE_TIMEOUT_MS 1000

// PID constants and rudder mixing constant for FBWA control law
static constexpr float AM_FBWA_ROLL_P_GAIN = 1.244f;
static constexpr float AM_FBWA_ROLL_I_GAIN = 0.590f;
static constexpr float AM_FBWA_ROLL_D_GAIN = 0.138f;
static constexpr float AM_FBWA_ROLL_D_TAU = 0.02f;
static constexpr float AM_FBWA_PITCH_P_GAIN = 2.240f;
static constexpr float AM_FBWA_PITCH_I_GAIN = 1.200f;
static constexpr float AM_FBWA_PITCH_D_GAIN = 0.282f;
static constexpr float AM_FBWA_PITCH_D_TAU = 0.02f;
static constexpr float AM_FBWA_RUDDER_MIXING = 0.5f;

typedef enum {
    YAW = 0,
    PITCH,
    ROLL,
    THROTTLE,
    FLAP_ANGLE,
    STEERING
} ControlAxis_t;

class AttitudeManager {
    public:
        AttitudeManager(
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
        );

        void amUpdate();

    private:
        ISystemUtils *systemUtilsDriver;

        IGPS *gpsDriver;
        IIMU *imuDriver;

        Mahony mahonyFilter;

        IMessageQueue<RCMotorControlMessage_t> *amQueue;
        IMessageQueue<TMMessage_t> *tmQueue;
        IMessageQueue<char[100]> *smLoggerQueue;
        char logBuf[100];

        Flightmode *activeCLAW;     // Pointer to current active Control Law
        DirectMapping manualCLAW;   // Manual Control Law (Direct Passthrough)
        FBWAMapping fbwaCLAW;       // Fly-By-Wire A Control Law (Roll and Pitch PID + Yaw Rudder Mixing)
        RCMotorControlMessage_t controlMsg;
        DroneState_t droneState;
        PlaneFlightMode_e currentFlightMode;

        MotorGroupInstance_t *rollMotors;
        MotorGroupInstance_t *pitchMotors;
        MotorGroupInstance_t *yawMotors;
        MotorGroupInstance_t *throttleMotors;
        MotorGroupInstance_t *flapMotors;
        MotorGroupInstance_t *steeringMotors;

        uint16_t lastServoOutputs[16];

        uint8_t amSchedulingCounter;

        int noDataCount;
        bool failsafeTriggered;

        bool getControlInputs(RCMotorControlMessage_t *pControlMsg);

        void outputToMotor(ControlAxis_t axis, uint8_t percent);

        void sendGPSDataToTelemetryManager(const GpsData_t &gpsData);
        void sendRawIMUDataToTelemetryManager(const RawImu_t &imuData);
        void sendAttitudeDataToTelemetryManager(const Attitude_t &attitude);
        void sendServoOutputRawToTelemetryManager();
};
