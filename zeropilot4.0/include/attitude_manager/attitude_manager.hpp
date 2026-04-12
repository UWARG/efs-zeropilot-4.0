#pragma once

#include <cstdint>
#include <utility>
#include "systemutils_iface.hpp"
#include "direct_mapping.hpp"
#include "fbwa_mapping.hpp"
#include "motor_datatype.hpp"
#include "gps_iface.hpp"
#include "tm_queue.hpp"
#include "imu_iface.hpp"
#include "MahonyAHRS.hpp"
#include "barometer_iface.hpp"
#include "queue_iface.hpp"
#include "drone_state.hpp"

#define AM_SCHEDULING_RATE_HZ 100
#define AM_TELEMETRY_GPS_DATA_RATE_HZ 5
#define AM_TELEMETRY_RAW_IMU_DATA_RATE_HZ 10
#define AM_TELEMETRY_ATTITUDE_DATA_RATE_HZ 20
#define AM_TELEMETRY_SERVO_OUTPUT_RAW_RATE_HZ 2

#define AM_UPDATE_LOOP_DELAY_MS (1000 / AM_SCHEDULING_RATE_HZ)
#define AM_CONTROL_LOOP_PERIOD_S (static_cast<float>(AM_UPDATE_LOOP_DELAY_MS) / 1000.0f)

class AttitudeManager {
    public:
        AttitudeManager(
            ISystemUtils *systemUtilsDriver,
            IGPS *gpsDriver,
            IIMU *imuDriver,
            IBarometer *barometerDriver,
            IMessageQueue<RCMotorControlMessage_t> *amQueue,
            IMessageQueue<TMMessage_t> *tmQueue,
            IMessageQueue<char[100]> *smLoggerQueue,
            MotorGroupInstance_t *mainMotorGroup
        );

        void amUpdate();

    private:
        ISystemUtils *systemUtilsDriver;

        IGPS *gpsDriver;
        IIMU *imuDriver;
        IBarometer *barometerDriver;

        Mahony mahonyFilter;

        IMessageQueue<RCMotorControlMessage_t> *amQueue;
        IMessageQueue<TMMessage_t> *tmQueue;
        IMessageQueue<char[100]> *smLoggerQueue;

        Flightmode *activeCLAW;     // Pointer to current active Control Law
        DirectMapping manualCLAW;   // Manual Control Law (Direct Passthrough)
        FBWAMapping fbwaCLAW;       // Fly-By-Wire A Control Law (Roll and Pitch PID + Yaw Rudder Mixing)
        RCMotorControlMessage_t controlMsg;
        DroneState_t droneState;
        PlaneFlightMode_e currentFlightMode;

        MotorGroupInstance_t *mainMotorGroup;

        bool armedFlag;

        uint16_t lastServoOutputs[16];

        uint8_t amSchedulingCounter;

        int noDataCount;
        bool failsafeTriggered;

        bool getControlInputs(RCMotorControlMessage_t *pControlMsg);

        void outputToMotors(RCMotorControlMessage_t outputControlMsg);

        void sendGPSDataToTelemetryManager(const GpsData_t &gpsData);
        void sendRawIMUDataToTelemetryManager(const RawImu_t &imuData);
        void sendAttitudeDataToTelemetryManager(const Attitude_t &attitude);
        void sendServoOutputRawToTelemetryManager();

        void loadServoParams();
        void bindServoParamCallbacks();

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

        // Servo param callbacks: one unique function is generated at compile time for each
        // servo parameter index (Idx). The compiler stamps out updateServoParam<0>,
        // updateServoParam<1>, ... so each ZP_PARAM slot gets its own callback with the
        // channel and field baked in as compile-time constants.
        template <uint8_t Idx> static bool updateServoParam(AttitudeManager* ctx, float val);

        // Registers all servo-param callbacks in one shot using C++ parameter pack expansion.
        // std::integer_sequence<uint8_t, Is...> carries the compile-time list {0, 1, 2, ...}
        // and the "..." expands a bindCallback() call for every index in that list.
        template <uint8_t... Is>
        void bindServoParamCallbacksImpl(std::integer_sequence<uint8_t, Is...>);
};
