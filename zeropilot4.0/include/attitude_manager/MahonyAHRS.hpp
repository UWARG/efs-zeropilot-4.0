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
#ifndef MAHONY_AHRS_H
#define MAHONY_AHRS_H
#include <math.h>
#include "imu_datatypes.hpp"

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
	static ZP_ERROR_e invSqrt(float x, float *output);
	bool isInitialized = false;

//-------------------------------------------------------------------------------------------
// Function declarations

public:
	Mahony();
	ZP_ERROR_e begin(float sampleFrequency) { 
		if (sampleFrequency <= 0) {
			return ZP_ERROR_INVALID_PARAM;
		}
		if (isInitialized) {
			return ZP_ERROR_ALREADY_INITIALIZED;
		}
		invSampleFreq = 1.0f / sampleFrequency; 
		isInitialized = true;
		return ZP_ERROR_OK;
	}

	ZP_ERROR_e updateIMU(float gx, float gy, float gz, float ax, float ay, float az);

	ZP_ERROR_e getAttitude(Attitude_t *attitude) {
		if (attitude == nullptr) {
			return ZP_ERROR_NULLPTR;
		}
		if (!isInitialized) {
			return ZP_ERROR_NOT_READY;
		}
		*attitude = Attitude_t{
			roll * 57.29578f,
			pitch * 57.29578f,
			yaw * 57.29578f + 180.0f
		};
		return ZP_ERROR_OK;
	}

	ZP_ERROR_e getAttitudeRadians(Attitude_t *attitude) {
		if (attitude == nullptr) {
			return ZP_ERROR_NULLPTR;
		}
		if (!isInitialized) {
			return ZP_ERROR_NOT_READY;
		}
		*attitude = Attitude_t{
			roll,
			pitch,
			yaw
		};
		return ZP_ERROR_OK;
	}
};

#endif
