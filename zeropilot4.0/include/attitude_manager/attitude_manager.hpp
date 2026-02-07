#pragma once

#include <cstdint>
#include "systemutils_iface.hpp"
#include "direct_mapping.hpp"
#include "motor_datatype.hpp"
#include "gps_iface.hpp"
#include "tm_queue.hpp"
#include "imu_iface.hpp"
#include "MahonyAHRS.hpp"
#include "queue_iface.hpp"
#include "drone_state.hpp"

#define AM_CONTROL_LOOP_DELAY 10
#define AM_CONTROL_LOOP_PERIOD_S (static_cast<float>(AM_CONTROL_LOOP_DELAY) / 1000.0f)
#define AM_FAILSAFE_TIMEOUT 1000

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

        DirectMapping controlAlgorithm;
        RCMotorControlMessage_t controlMsg;
        DroneState_t droneState;

        MotorGroupInstance_t *rollMotors;
        MotorGroupInstance_t *pitchMotors;
        MotorGroupInstance_t *yawMotors;
        MotorGroupInstance_t *throttleMotors;
        MotorGroupInstance_t *flapMotors;
        MotorGroupInstance_t *steeringMotors;

        uint8_t amSchedulingCounter;

        bool getControlInputs(RCMotorControlMessage_t *pControlMsg);

        void outputToMotor(ControlAxis_t axis, uint8_t percent);

        void sendGPSDataToTelemetryManager(const GpsData_t &gpsData);

        void sendRawIMUDataToTelemetryManager(const RawImu_t &imuData);

        void sendAttitudeDataToTelemetryManager(const Attitude_t &attitude);
};
