//=============================================================================================
// MahonyAHRS.h
//=============================================================================================
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
//
// Date			Author			Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=============================================================================================
#pragma once
#include <math.h>
#include "imu_datatypes.hpp"
#include "zp_error.h"
//--------------------------------------------------------------------------------------------
// Variable declaration

class Mahony {
private:
	float twoKp;		// 2 * proportional gain (Kp)
	float twoKi;		// 2 * integral gain (Ki)
	float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame
	float integralFBx, integralFBy, integralFBz;  // integral error terms scaled by Ki
	float invSampleFreq;
	float roll, pitch, yaw;
	static ZP_ERROR_e invSqrt(float x, float &output);
	bool isInitialized = false;

//-------------------------------------------------------------------------------------------
// Function declarations

public:
	Mahony();
	// Initializer
    ZP_ERROR_e begin(float sampleFrequency) { 
        ZP_ERROR_e result = ZP_ERROR_OK;

        if (sampleFrequency <= 0.0f) {
            result |= ZP_ERROR_INVALID_PARAM;
        }
        if (isInitialized) {
            result |= ZP_ERROR_ALREADY_INITIALIZED;
        }

        if (result == ZP_ERROR_OK) {
            invSampleFreq = 1.0f / sampleFrequency; 
            isInitialized = true;
        }
        return result;
    }

	ZP_ERROR_e updateIMU(float gx, float gy, float gz, float ax, float ay, float az);

	ZP_ERROR_e getAttitude(Attitude_t& out_attitude) {
        ZP_ERROR_e result = ZP_ERROR_OK;
   
        if (!isInitialized) {
            result |= ZP_ERROR_NOT_READY;
        }
        
        if (result == ZP_ERROR_OK) {
            out_attitude.roll = roll * 57.29578f;
            out_attitude.pitch = pitch * 57.29578f;
            out_attitude.yaw = yaw * 57.29578f + 180.0f;
        }
        return result;
    }

	ZP_ERROR_e getAttitudeRadians(Attitude_t& out_attitude) {
        ZP_ERROR_e result = ZP_ERROR_OK;
   
        if (!isInitialized) {
            result |= ZP_ERROR_NOT_READY;
        }
        
        if (result == ZP_ERROR_OK) {
            out_attitude.roll = roll;
            out_attitude.pitch = pitch;
            out_attitude.yaw = yaw;
        }
        return result;
    }
};