#include "zp_params.hpp"
#include "mavlink.h"
#include "flightmode.hpp"
#include "motor_functions.hpp"
#include <cstring>

namespace ZP_PARAM {

    static Param_t params[static_cast<uint16_t>(ZP_PARAM_ID::PARAM_COUNT)];

    namespace {
        static ZP_Error initParam(ZP_PARAM_ID id, const char* name, float default_val, uint8_t type);
    }

    ZP_Error init() {
        ZP_Error result = ZP_ERROR_OK;
        std::memset(params, 0, sizeof(params));

        // Define your parameter set
        result |= initParam(ZP_PARAM_ID::SERVO1_TRIM, "SERVO1_TRIM", 1500, MAV_PARAM_TYPE_UINT16);
        result |= initParam(ZP_PARAM_ID::SERVO1_MIN, "SERVO1_MIN", 1000, MAV_PARAM_TYPE_UINT16);
        result |= initParam(ZP_PARAM_ID::SERVO1_MAX, "SERVO1_MAX", 2000, MAV_PARAM_TYPE_UINT16);
        #ifdef PLANE
        result |= initParam(ZP_PARAM_ID::SERVO1_REVERSED, "SERVO1_REVERSED", 1, MAV_PARAM_TYPE_UINT8);
        result |= initParam(ZP_PARAM_ID::SERVO1_FUNCTION, "SERVO1_FUNCTION", static_cast<float>(MotorFunction_e::AILERON), MAV_PARAM_TYPE_INT16);
        #endif
        #ifdef QUADCOPTER
        result |= initParam(ZP_PARAM_ID::SERVO1_REVERSED, "SERVO1_REVERSED", 0, MAV_PARAM_TYPE_UINT8);
        result |= initParam(ZP_PARAM_ID::SERVO1_FUNCTION, "SERVO1_FUNCTION", static_cast<float>(MotorFunction_e::MOTOR_1), MAV_PARAM_TYPE_INT16);
        #endif

        result |= initParam(ZP_PARAM_ID::SERVO2_TRIM, "SERVO2_TRIM", 1500, MAV_PARAM_TYPE_UINT16);
        result |= initParam(ZP_PARAM_ID::SERVO2_MIN, "SERVO2_MIN", 1000, MAV_PARAM_TYPE_UINT16);
        result |= initParam(ZP_PARAM_ID::SERVO2_MAX, "SERVO2_MAX", 2000, MAV_PARAM_TYPE_UINT16);
        #ifdef PLANE
        result |= initParam(ZP_PARAM_ID::SERVO2_REVERSED, "SERVO2_REVERSED", 0, MAV_PARAM_TYPE_UINT8);
        result |= initParam(ZP_PARAM_ID::SERVO2_FUNCTION, "SERVO2_FUNCTION", static_cast<float>(MotorFunction_e::ELEVATOR), MAV_PARAM_TYPE_INT16);
        #endif
        #ifdef QUADCOPTER
        result |= initParam(ZP_PARAM_ID::SERVO2_REVERSED, "SERVO2_REVERSED", 0, MAV_PARAM_TYPE_UINT8);
        result |= initParam(ZP_PARAM_ID::SERVO2_FUNCTION, "SERVO2_FUNCTION", static_cast<float>(MotorFunction_e::MOTOR_2), MAV_PARAM_TYPE_INT16);
        #endif

        result |= initParam(ZP_PARAM_ID::SERVO3_TRIM, "SERVO3_TRIM", 1500, MAV_PARAM_TYPE_UINT16);
        result |= initParam(ZP_PARAM_ID::SERVO3_MIN, "SERVO3_MIN", 1000, MAV_PARAM_TYPE_UINT16);
        result |= initParam(ZP_PARAM_ID::SERVO3_MAX, "SERVO3_MAX", 2000, MAV_PARAM_TYPE_UINT16);
        #ifdef PLANE
        result |= initParam(ZP_PARAM_ID::SERVO3_REVERSED, "SERVO3_REVERSED", 0, MAV_PARAM_TYPE_UINT8);
        result |= initParam(ZP_PARAM_ID::SERVO3_FUNCTION, "SERVO3_FUNCTION", static_cast<float>(MotorFunction_e::THROTTLE), MAV_PARAM_TYPE_INT16);
        #endif
        #ifdef QUADCOPTER
        result |= initParam(ZP_PARAM_ID::SERVO3_REVERSED, "SERVO3_REVERSED", 0, MAV_PARAM_TYPE_UINT8);
        result |= initParam(ZP_PARAM_ID::SERVO3_FUNCTION, "SERVO3_FUNCTION", static_cast<float>(MotorFunction_e::MOTOR_3), MAV_PARAM_TYPE_INT16);
        #endif

        result |= initParam(ZP_PARAM_ID::SERVO4_TRIM, "SERVO4_TRIM", 1500, MAV_PARAM_TYPE_UINT16);
        result |= initParam(ZP_PARAM_ID::SERVO4_MIN, "SERVO4_MIN", 1000, MAV_PARAM_TYPE_UINT16);
        result |= initParam(ZP_PARAM_ID::SERVO4_MAX, "SERVO4_MAX", 2000, MAV_PARAM_TYPE_UINT16);
        #ifdef PLANE
        result |= initParam(ZP_PARAM_ID::SERVO4_REVERSED, "SERVO4_REVERSED", 0, MAV_PARAM_TYPE_UINT8);
        result |= initParam(ZP_PARAM_ID::SERVO4_FUNCTION, "SERVO4_FUNCTION", static_cast<float>(MotorFunction_e::RUDDER), MAV_PARAM_TYPE_INT16);
        #endif
        #ifdef QUADCOPTER
        result |= initParam(ZP_PARAM_ID::SERVO4_REVERSED, "SERVO4_REVERSED", 0, MAV_PARAM_TYPE_UINT8);
        result |= initParam(ZP_PARAM_ID::SERVO4_FUNCTION, "SERVO4_FUNCTION", static_cast<float>(MotorFunction_e::MOTOR_4), MAV_PARAM_TYPE_INT16);
        #endif

        result |= initParam(ZP_PARAM_ID::SERVO5_TRIM, "SERVO5_TRIM", 1500, MAV_PARAM_TYPE_UINT16);
        result |= initParam(ZP_PARAM_ID::SERVO5_MIN, "SERVO5_MIN", 1000, MAV_PARAM_TYPE_UINT16);
        result |= initParam(ZP_PARAM_ID::SERVO5_MAX, "SERVO5_MAX", 2000, MAV_PARAM_TYPE_UINT16);
        #ifdef PLANE
        result |= initParam(ZP_PARAM_ID::SERVO5_REVERSED, "SERVO5_REVERSED", 1, MAV_PARAM_TYPE_UINT8);
        result |= initParam(ZP_PARAM_ID::SERVO5_FUNCTION, "SERVO5_FUNCTION", static_cast<float>(MotorFunction_e::AILERON), MAV_PARAM_TYPE_INT16);
        #endif
        #ifdef QUADCOPTER
        result |= initParam(ZP_PARAM_ID::SERVO5_REVERSED, "SERVO5_REVERSED", 0, MAV_PARAM_TYPE_UINT8);
        result |= initParam(ZP_PARAM_ID::SERVO5_FUNCTION, "SERVO5_FUNCTION", static_cast<float>(MotorFunction_e::DISABLED), MAV_PARAM_TYPE_INT16);
        #endif

        result |= initParam(ZP_PARAM_ID::SERVO6_TRIM, "SERVO6_TRIM", 1500, MAV_PARAM_TYPE_UINT16);
        result |= initParam(ZP_PARAM_ID::SERVO6_MIN, "SERVO6_MIN", 1000, MAV_PARAM_TYPE_UINT16);
        result |= initParam(ZP_PARAM_ID::SERVO6_MAX, "SERVO6_MAX", 2000, MAV_PARAM_TYPE_UINT16);
        #ifdef PLANE
        result |= initParam(ZP_PARAM_ID::SERVO6_REVERSED, "SERVO6_REVERSED", 0, MAV_PARAM_TYPE_UINT8);
        result |= initParam(ZP_PARAM_ID::SERVO6_FUNCTION, "SERVO6_FUNCTION", static_cast<float>(MotorFunction_e::FLAP), MAV_PARAM_TYPE_INT16);
        #endif
        #ifdef QUADCOPTER
        result |= initParam(ZP_PARAM_ID::SERVO6_REVERSED, "SERVO6_REVERSED", 0, MAV_PARAM_TYPE_UINT8);
        result |= initParam(ZP_PARAM_ID::SERVO6_FUNCTION, "SERVO6_FUNCTION", static_cast<float>(MotorFunction_e::DISABLED), MAV_PARAM_TYPE_INT16);
        #endif

        result |= initParam(ZP_PARAM_ID::SERVO7_TRIM, "SERVO7_TRIM", 1500, MAV_PARAM_TYPE_UINT16);
        result |= initParam(ZP_PARAM_ID::SERVO7_MIN, "SERVO7_MIN", 1000, MAV_PARAM_TYPE_UINT16);
        result |= initParam(ZP_PARAM_ID::SERVO7_MAX, "SERVO7_MAX", 2000, MAV_PARAM_TYPE_UINT16);
        #ifdef PLANE
        result |= initParam(ZP_PARAM_ID::SERVO7_REVERSED, "SERVO7_REVERSED", 0, MAV_PARAM_TYPE_UINT8);
        result |= initParam(ZP_PARAM_ID::SERVO7_FUNCTION, "SERVO7_FUNCTION", static_cast<float>(MotorFunction_e::FLAP), MAV_PARAM_TYPE_INT16);
        #endif
        #ifdef QUADCOPTER
        result |= initParam(ZP_PARAM_ID::SERVO7_REVERSED, "SERVO7_REVERSED", 0, MAV_PARAM_TYPE_UINT8);
        result |= initParam(ZP_PARAM_ID::SERVO7_FUNCTION, "SERVO7_FUNCTION", static_cast<float>(MotorFunction_e::DISABLED), MAV_PARAM_TYPE_INT16);
        #endif

        result |= initParam(ZP_PARAM_ID::SERVO8_TRIM, "SERVO8_TRIM", 1500, MAV_PARAM_TYPE_UINT16);
        result |= initParam(ZP_PARAM_ID::SERVO8_MIN, "SERVO8_MIN", 1000, MAV_PARAM_TYPE_UINT16);
        result |= initParam(ZP_PARAM_ID::SERVO8_MAX, "SERVO8_MAX", 2000, MAV_PARAM_TYPE_UINT16);
        result |= initParam(ZP_PARAM_ID::SERVO8_REVERSED, "SERVO8_REVERSED", 0, MAV_PARAM_TYPE_UINT8);
        #ifdef PLANE
        result |= initParam(ZP_PARAM_ID::SERVO8_REVERSED, "SERVO8_REVERSED", 0, MAV_PARAM_TYPE_UINT8);
        result |= initParam(ZP_PARAM_ID::SERVO8_FUNCTION, "SERVO8_FUNCTION", static_cast<float>(MotorFunction_e::GROUND_STEERING), MAV_PARAM_TYPE_INT16);
        #endif
        #ifdef QUADCOPTER
        result |= initParam(ZP_PARAM_ID::SERVO8_REVERSED, "SERVO8_REVERSED", 0, MAV_PARAM_TYPE_UINT8);
        result |= initParam(ZP_PARAM_ID::SERVO8_FUNCTION, "SERVO8_FUNCTION", static_cast<float>(MotorFunction_e::DISABLED), MAV_PARAM_TYPE_INT16);
        #endif

        result |= initParam(ZP_PARAM_ID::SERVO9_TRIM, "SERVO9_TRIM", 1500, MAV_PARAM_TYPE_UINT16);
        result |= initParam(ZP_PARAM_ID::SERVO9_MIN, "SERVO9_MIN", 1000, MAV_PARAM_TYPE_UINT16);
        result |= initParam(ZP_PARAM_ID::SERVO9_MAX, "SERVO9_MAX", 2000, MAV_PARAM_TYPE_UINT16);
        result |= initParam(ZP_PARAM_ID::SERVO9_REVERSED, "SERVO9_REVERSED", 0, MAV_PARAM_TYPE_UINT8);
        result |= initParam(ZP_PARAM_ID::SERVO9_FUNCTION, "SERVO9_FUNCTION", static_cast<float>(MotorFunction_e::DISABLED), MAV_PARAM_TYPE_INT16);

        result |= initParam(ZP_PARAM_ID::SERVO10_TRIM, "SERVO10_TRIM", 1500, MAV_PARAM_TYPE_UINT16);
        result |= initParam(ZP_PARAM_ID::SERVO10_MIN, "SERVO10_MIN", 1000, MAV_PARAM_TYPE_UINT16);
        result |= initParam(ZP_PARAM_ID::SERVO10_MAX, "SERVO10_MAX", 2000, MAV_PARAM_TYPE_UINT16);
        result |= initParam(ZP_PARAM_ID::SERVO10_REVERSED, "SERVO10_REVERSED", 0, MAV_PARAM_TYPE_UINT8);
        result |= initParam(ZP_PARAM_ID::SERVO10_FUNCTION, "SERVO10_FUNCTION", static_cast<float>(MotorFunction_e::DISABLED), MAV_PARAM_TYPE_INT16);

        result |= initParam(ZP_PARAM_ID::SERVO11_TRIM, "SERVO11_TRIM", 1500, MAV_PARAM_TYPE_UINT16);
        result |= initParam(ZP_PARAM_ID::SERVO11_MIN, "SERVO11_MIN", 1000, MAV_PARAM_TYPE_UINT16);
        result |= initParam(ZP_PARAM_ID::SERVO11_MAX, "SERVO11_MAX", 2000, MAV_PARAM_TYPE_UINT16);
        result |= initParam(ZP_PARAM_ID::SERVO11_REVERSED, "SERVO11_REVERSED", 0, MAV_PARAM_TYPE_UINT8);
        result |= initParam(ZP_PARAM_ID::SERVO11_FUNCTION, "SERVO11_FUNCTION", static_cast<float>(MotorFunction_e::DISABLED), MAV_PARAM_TYPE_INT16);

        result |= initParam(ZP_PARAM_ID::SERVO12_TRIM, "SERVO12_TRIM", 1500, MAV_PARAM_TYPE_UINT16);
        result |= initParam(ZP_PARAM_ID::SERVO12_MIN, "SERVO12_MIN", 1000, MAV_PARAM_TYPE_UINT16);
        result |= initParam(ZP_PARAM_ID::SERVO12_MAX, "SERVO12_MAX", 2000, MAV_PARAM_TYPE_UINT16);
        result |= initParam(ZP_PARAM_ID::SERVO12_REVERSED, "SERVO12_REVERSED", 0, MAV_PARAM_TYPE_UINT8);
        result |= initParam(ZP_PARAM_ID::SERVO12_FUNCTION, "SERVO12_FUNCTION", static_cast<float>(MotorFunction_e::DISABLED), MAV_PARAM_TYPE_INT16);

        #ifdef PLANE
        result |= initParam(ZP_PARAM_ID::MOT_PWM_TYPE, "MOT_PWM_TYPE", 0, MAV_PARAM_TYPE_UINT16);

        result |= initParam(ZP_PARAM_ID::RLL2SRV_P, "RLL2SRV_P", 0.5f, MAV_PARAM_TYPE_REAL32);
        result |= initParam(ZP_PARAM_ID::RLL2SRV_I, "RLL2SRV_I", 0.2f, MAV_PARAM_TYPE_REAL32);
        result |= initParam(ZP_PARAM_ID::RLL2SRV_D, "RLL2SRV_D", 0.05f, MAV_PARAM_TYPE_REAL32);
        result |= initParam(ZP_PARAM_ID::RLL2SRV_TAU, "RLL2SRV_TAU", 0.020f, MAV_PARAM_TYPE_REAL32);
        result |= initParam(ZP_PARAM_ID::RLL2SRV_IMAX, "RLL2SRV_IMAX", 50, MAV_PARAM_TYPE_UINT8);
        result |= initParam(ZP_PARAM_ID::RLL2SRV_FF, "RLL2SRV_FF", 0.0f, MAV_PARAM_TYPE_REAL32);

        result |= initParam(ZP_PARAM_ID::PTCH2SRV_P, "PTCH2SRV_P", 1.2f, MAV_PARAM_TYPE_REAL32);
        result |= initParam(ZP_PARAM_ID::PTCH2SRV_I, "PTCH2SRV_I", 0.6f, MAV_PARAM_TYPE_REAL32);
        result |= initParam(ZP_PARAM_ID::PTCH2SRV_D, "PTCH2SRV_D", 0.08f, MAV_PARAM_TYPE_REAL32);
        result |= initParam(ZP_PARAM_ID::PTCH2SRV_TAU, "PTCH2SRV_TAU", 0.020f, MAV_PARAM_TYPE_REAL32);
        result |= initParam(ZP_PARAM_ID::PTCH2SRV_IMAX, "PTCH2SRV_IMAX", 50, MAV_PARAM_TYPE_UINT8);
        result |= initParam(ZP_PARAM_ID::PTCH2SRV_FF, "PTCH2SRV_FF", 0.0f, MAV_PARAM_TYPE_REAL32);

        result |= initParam(ZP_PARAM_ID::KFF_RDDRMIX, "KFF_RDDRMIX", 0.500f, MAV_PARAM_TYPE_REAL32);

        result |= initParam(ZP_PARAM_ID::ROLL_LIMIT_DEG, "ROLL_LIMIT_DEG", 45.0f, MAV_PARAM_TYPE_REAL32);
        result |= initParam(ZP_PARAM_ID::PTCH_LIM_MAX_DEG, "PTCH_LIM_MAX_DEG", 20.0f, MAV_PARAM_TYPE_REAL32);
        result |= initParam(ZP_PARAM_ID::PTCH_LIM_MIN_DEG, "PTCH_LIM_MIN_DEG", -20.0f, MAV_PARAM_TYPE_REAL32);

        result |= initParam(ZP_PARAM_ID::FLTMODE1, "FLTMODE1", static_cast<float>(FlightMode_e::MANUAL), MAV_PARAM_TYPE_UINT32);
        result |= initParam(ZP_PARAM_ID::FLTMODE2, "FLTMODE2", static_cast<float>(FlightMode_e::FBWA),   MAV_PARAM_TYPE_UINT32);
        result |= initParam(ZP_PARAM_ID::FLTMODE3, "FLTMODE3", static_cast<float>(FlightMode_e::MANUAL), MAV_PARAM_TYPE_UINT32);
        result |= initParam(ZP_PARAM_ID::FLTMODE4, "FLTMODE4", static_cast<float>(FlightMode_e::MANUAL), MAV_PARAM_TYPE_UINT32);
        result |= initParam(ZP_PARAM_ID::FLTMODE5, "FLTMODE5", static_cast<float>(FlightMode_e::MANUAL), MAV_PARAM_TYPE_UINT32);
        result |= initParam(ZP_PARAM_ID::FLTMODE6, "FLTMODE6", static_cast<float>(FlightMode_e::MANUAL), MAV_PARAM_TYPE_UINT32);
        #endif

        #ifdef QUADCOPTER
        result |= initParam(ZP_PARAM_ID::MOT_PWM_TYPE, "MOT_PWM_TYPE", 5, MAV_PARAM_TYPE_UINT16);

        result |= initParam(ZP_PARAM_ID::MOT_SPIN_MIN, "MOT_SPIN_MIN", 0.15f, MAV_PARAM_TYPE_REAL32);
        result |= initParam(ZP_PARAM_ID::MOT_SPIN_MAX, "MOT_SPIN_MAX", 0.95f, MAV_PARAM_TYPE_REAL32);
        result |= initParam(ZP_PARAM_ID::MOT_SPIN_ARM, "MOT_SPIN_ARM", 0.05f, MAV_PARAM_TYPE_REAL32);

        result |= initParam(ZP_PARAM_ID::ATC_RAT_RLL_P, "ATC_RAT_RLL_P", 0.140f, MAV_PARAM_TYPE_REAL32);
        result |= initParam(ZP_PARAM_ID::ATC_RAT_RLL_I, "ATC_RAT_RLL_I", 0.140f, MAV_PARAM_TYPE_REAL32);
        result |= initParam(ZP_PARAM_ID::ATC_RAT_RLL_D, "ATC_RAT_RLL_D", 0.0025f, MAV_PARAM_TYPE_REAL32);
        result |= initParam(ZP_PARAM_ID::ATC_RAT_RLL_TAU, "ATC_RAT_RLL_TAU", 0.020f, MAV_PARAM_TYPE_REAL32);
        result |= initParam(ZP_PARAM_ID::ATC_RAT_RLL_IMAX, "ATC_RAT_RLL_IMAX", 50, MAV_PARAM_TYPE_UINT8);

        result |= initParam(ZP_PARAM_ID::ATC_RAT_PIT_P, "ATC_RAT_PIT_P", 0.140f, MAV_PARAM_TYPE_REAL32);
        result |= initParam(ZP_PARAM_ID::ATC_RAT_PIT_I, "ATC_RAT_PIT_I", 0.140f, MAV_PARAM_TYPE_REAL32);
        result |= initParam(ZP_PARAM_ID::ATC_RAT_PIT_D, "ATC_RAT_PIT_D", 0.0025f, MAV_PARAM_TYPE_REAL32);
        result |= initParam(ZP_PARAM_ID::ATC_RAT_PIT_TAU, "ATC_RAT_PIT_TAU", 0.020f, MAV_PARAM_TYPE_REAL32);
        result |= initParam(ZP_PARAM_ID::ATC_RAT_PIT_IMAX, "ATC_RAT_PIT_IMAX", 50, MAV_PARAM_TYPE_UINT8);

        result |= initParam(ZP_PARAM_ID::ATC_RAT_YAW_P, "ATC_RAT_YAW_P", 0.3438f, MAV_PARAM_TYPE_REAL32);
        result |= initParam(ZP_PARAM_ID::ATC_RAT_YAW_I, "ATC_RAT_YAW_I", 0.3438f, MAV_PARAM_TYPE_REAL32);
        result |= initParam(ZP_PARAM_ID::ATC_RAT_YAW_D, "ATC_RAT_YAW_D", 0.0f, MAV_PARAM_TYPE_REAL32);
        result |= initParam(ZP_PARAM_ID::ATC_RAT_YAW_TAU, "ATC_RAT_YAW_TAU", 0.020f, MAV_PARAM_TYPE_REAL32);
        result |= initParam(ZP_PARAM_ID::ATC_RAT_YAW_IMAX, "ATC_RAT_YAW_IMAX", 50, MAV_PARAM_TYPE_UINT8);

        result |= initParam(ZP_PARAM_ID::ACRO_RP_RATE, "ACRO_RP_RATE", 360.0f, MAV_PARAM_TYPE_REAL32);
        result |= initParam(ZP_PARAM_ID::ACRO_Y_RATE, "ACRO_Y_RATE", 202.5f, MAV_PARAM_TYPE_REAL32);

        result |= initParam(ZP_PARAM_ID::ATC_ANG_RLL_P, "ATC_ANG_RLL_P", 0.7f, MAV_PARAM_TYPE_REAL32);
        result |= initParam(ZP_PARAM_ID::ATC_ANG_RLL_I, "ATC_ANG_RLL_I", 0.0f, MAV_PARAM_TYPE_REAL32);
        result |= initParam(ZP_PARAM_ID::ATC_ANG_RLL_D, "ATC_ANG_RLL_D", 0.0f, MAV_PARAM_TYPE_REAL32);
        result |= initParam(ZP_PARAM_ID::ATC_ANG_RLL_TAU, "ATC_ANG_RLL_TAU", 0.020f, MAV_PARAM_TYPE_REAL32);
        result |= initParam(ZP_PARAM_ID::ATC_ANG_RLL_IMAX, "ATC_ANG_RLL_IMAX", 50, MAV_PARAM_TYPE_UINT8);

        result |= initParam(ZP_PARAM_ID::ATC_ANG_PTCH_P, "ATC_ANG_PTCH_P", 0.7f, MAV_PARAM_TYPE_REAL32);
        result |= initParam(ZP_PARAM_ID::ATC_ANG_PTCH_I, "ATC_ANG_PTCH_I", 0.0f, MAV_PARAM_TYPE_REAL32);
        result |= initParam(ZP_PARAM_ID::ATC_ANG_PTCH_D, "ATC_ANG_PTCH_D", 0.0f, MAV_PARAM_TYPE_REAL32);
        result |= initParam(ZP_PARAM_ID::ATC_ANG_PTCH_TAU, "ATC_ANG_PTCH_TAU", 0.020f, MAV_PARAM_TYPE_REAL32);
        result |= initParam(ZP_PARAM_ID::ATC_ANG_PTCH_IMAX, "ATC_ANG_PTCH_IMAX", 50, MAV_PARAM_TYPE_UINT8);

        result |= initParam(ZP_PARAM_ID::ATC_ANGLE_MAX, "ATC_ANGLE_MAX", 30.0f, MAV_PARAM_TYPE_REAL32);

        result |= initParam(ZP_PARAM_ID::FLTMODE1, "FLTMODE1", static_cast<float>(FlightMode_e::STABILIZE), MAV_PARAM_TYPE_UINT32);
        result |= initParam(ZP_PARAM_ID::FLTMODE2, "FLTMODE2", static_cast<float>(FlightMode_e::ACRO), MAV_PARAM_TYPE_UINT32);
        result |= initParam(ZP_PARAM_ID::FLTMODE3, "FLTMODE3", static_cast<float>(FlightMode_e::ACRO), MAV_PARAM_TYPE_UINT32);
        result |= initParam(ZP_PARAM_ID::FLTMODE4, "FLTMODE4", static_cast<float>(FlightMode_e::ACRO), MAV_PARAM_TYPE_UINT32);
        result |= initParam(ZP_PARAM_ID::FLTMODE5, "FLTMODE5", static_cast<float>(FlightMode_e::ACRO), MAV_PARAM_TYPE_UINT32);
        result |= initParam(ZP_PARAM_ID::FLTMODE6, "FLTMODE6", static_cast<float>(FlightMode_e::ACRO), MAV_PARAM_TYPE_UINT32);
        #endif

        result |= initParam(ZP_PARAM_ID::RC_FS_TIMEOUT, "RC_FS_TIMEOUT", 0.5f, MAV_PARAM_TYPE_REAL32);

        result |= initParam(ZP_PARAM_ID::BATT_LOW_VOLT, "BATT_LOW_VOLT", 10.5f, MAV_PARAM_TYPE_REAL32);
        result |= initParam(ZP_PARAM_ID::BATT_CRT_VOLT, "BATT_CRT_VOLT", 10.2f, MAV_PARAM_TYPE_REAL32);
        result |= initParam(ZP_PARAM_ID::BATT_CAPACITY, "BATT_CAPACITY", 4000.0f, MAV_PARAM_TYPE_REAL32);
        result |= initParam(ZP_PARAM_ID::BATT_LOW_TIMER, "BATT_LOW_TIMER", 5.0f, MAV_PARAM_TYPE_REAL32);

        result |= initParam(ZP_PARAM_ID::RC1_REVERSED, "RC1_REVERSED", 0, MAV_PARAM_TYPE_UINT8);
        result |= initParam(ZP_PARAM_ID::RC2_REVERSED, "RC2_REVERSED", 1, MAV_PARAM_TYPE_UINT8);
        result |= initParam(ZP_PARAM_ID::RC3_REVERSED, "RC3_REVERSED", 0, MAV_PARAM_TYPE_UINT8);
        result |= initParam(ZP_PARAM_ID::RC4_REVERSED, "RC4_REVERSED", 0, MAV_PARAM_TYPE_UINT8);

        result |= initParam(ZP_PARAM_ID::BATT_N_CELLS, "BATT_N_CELLS", 3, MAV_PARAM_TYPE_INT8);

        result |= initParam(ZP_PARAM_ID::FFT_ENABLE, "FFT_ENABLE", 1, MAV_PARAM_TYPE_UINT8);
        result |= initParam(ZP_PARAM_ID::FFT_WINDOW_LEN, "FFT_WINDOW_LEN", 256, MAV_PARAM_TYPE_UINT16);
        result |= initParam(ZP_PARAM_ID::FFT_MINHZ, "FFT_MINHZ", 80.0f, MAV_PARAM_TYPE_REAL32);
        result |= initParam(ZP_PARAM_ID::INS_HNTCH_BW, "INS_HNTCH_BW", 30.0f, MAV_PARAM_TYPE_REAL32);
        result |= initParam(ZP_PARAM_ID::INS_HNTCH_ATT, "INS_HNTCH_ATT", 30.0f, MAV_PARAM_TYPE_REAL32);
        result |= initParam(ZP_PARAM_ID::INS_HNTCH_HMNCS, "INS_HNTCH_HMNCS", 0x0007, MAV_PARAM_TYPE_UINT16);

        result |= initParam(ZP_PARAM_ID::RNGFND_ENABLE, "RNGFND_ENABLE", 1, MAV_PARAM_TYPE_UINT8);
        result |= initParam(ZP_PARAM_ID::RNGFND_MIN, "RNGFND_MIN", 0.1f, MAV_PARAM_TYPE_REAL32);
        result |= initParam(ZP_PARAM_ID::RNGFND_MAX, "RNGFND_MAX", 20.0f, MAV_PARAM_TYPE_REAL32);
        return result;
    }

    ZP_Error bindCallbackInternal(ZP_PARAM_ID id, void* context, ParamSetterCb_t setter) {
        ZP_Error result = ZP_ERROR_OK;
        uint16_t index = static_cast<uint16_t>(id);

        if (index >= static_cast<uint16_t>(ZP_PARAM_ID::PARAM_COUNT)) {
            result |= ZP_ERROR_INVALID_ARG;
        } else {
            params[index].context = context;
            params[index].setter = setter;
        }
        
        return result;
    }

    ZP_Error get(ZP_PARAM_ID id, float& out_value) {
        uint16_t index = static_cast<uint16_t>(id);

        if (index >= static_cast<uint16_t>(ZP_PARAM_ID::PARAM_COUNT)) {
            out_value = 0.0f;
            return ZP_ERROR_INVALID_ARG;
        }
        out_value = params[index].paramValue;
        return ZP_ERROR_OK;
    }

    ZP_Error setParamById(const char* paramId, float new_value) {
        ZP_Error result = ZP_ERROR_OK;
        bool found = false;

        if (paramId == nullptr) {
            result |= ZP_ERROR_NULLPTR;
        } else {
            for (uint16_t i = 0; i < static_cast<uint16_t>(ZP_PARAM_ID::PARAM_COUNT); ++i) {
                if (std::strncmp(params[i].paramId, paramId, PARAM_MAX_IDENTIFIER_LEN - 1) == 0) {
                    found = true;
                    
                    if (params[i].setter != nullptr) {
                        result |= params[i].setter(params[i].context, new_value);
                    }

                    if (result == ZP_ERROR_OK) {
                        params[i].paramValue = new_value;
                    }
                    break;
                }
            }

            if (!found) {
                result |= ZP_ERROR_FAIL;
            }
        }

        return result;
    }

    ZP_Error getParamByIndex(uint16_t index, Param_t*& out_param) {
        ZP_Error result = ZP_ERROR_OK;

        if (index >= static_cast<uint16_t>(ZP_PARAM_ID::PARAM_COUNT)) {
            out_param = nullptr;
            result |= ZP_ERROR_INVALID_ARG;
        } else {
            out_param = &params[index];
        }
        
        return result;
    }

    ZP_Error getIndexById(const char* paramId, int16_t& out_index) {
        ZP_Error result = ZP_ERROR_OK;
        bool found = false;

        if (paramId == nullptr) {
            out_index = -1;
            result |= ZP_ERROR_NULLPTR;
        } else {
            for (uint16_t i = 0; i < static_cast<uint16_t>(ZP_PARAM_ID::PARAM_COUNT); ++i) {
                if (std::strncmp(params[i].paramId, paramId, PARAM_MAX_IDENTIFIER_LEN - 1) == 0) {
                    out_index = static_cast<int16_t>(i);
                    found = true;
                    break;
                }
            }

            if (!found) {
                out_index = -1;
                result |= ZP_ERROR_FAIL;
            }
        }

        return result;
    }

    uint16_t getCount() {
        return static_cast<uint16_t>(ZP_PARAM_ID::PARAM_COUNT);
    }


    namespace {
        static ZP_Error initParam(ZP_PARAM_ID id, const char* name, float default_val, uint8_t type) {
            ZP_Error result = ZP_ERROR_OK;
            uint16_t index = static_cast<uint16_t>(id);
    
            if (name == nullptr) {
                result |= ZP_ERROR_NULLPTR;
            } 
            
            if (index >= static_cast<uint16_t>(ZP_PARAM_ID::PARAM_COUNT)) {
                result |= ZP_ERROR_INVALID_ARG;
            }
    
            // Only proceed if no error bits have been set
            if (result == ZP_ERROR_OK) {
                std::strncpy(params[index].paramId, name, PARAM_MAX_IDENTIFIER_LEN);
                params[index].paramId[PARAM_MAX_IDENTIFIER_LEN - 1] = '\0';
                
                params[index].paramValue = default_val;
                params[index].paramType = type;
                params[index].context = nullptr;
                params[index].setter = nullptr;
            }
    
            return result;
        }
    }

}