//=============================================================================================
// MahonyAHRS.cpp
//=============================================================================================
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
//
// From the x-io website "Open-source resources available on this website are
// provided under the GNU General Public Licence unless an alternative licence
// is provided in source."
//
// Date			Author			Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
// Algorithm paper:
// http://ieeexplore.ieee.org/xpl/login.jsp?tp=&arnumber=4608934&url=http%3A%2F%2Fieeexplore.ieee.org%2Fstamp%2Fstamp.jsp%3Ftp%3D%26arnumber%3D4608934
//
//=============================================================================================

//-------------------------------------------------------------------------------------------
// Header files

#include "MahonyAHRS.hpp"
#include <math.h>
#include <zp_error.h>

//-------------------------------------------------------------------------------------------
// Definitions

#define DEFAULT_SAMPLE_FREQ	512.0f	// sample frequency in Hz
#define TWO_KP_DEF	(1.0f)	// proportional gain
#define TWO_KI_DEF	(0.05f)	// integral gain


//============================================================================================
// Functions

//-------------------------------------------------------------------------------------------
// AHRS algorithm update

Mahony::Mahony()
{
	twoKp = TWO_KP_DEF;	// 2 * proportional gain (Kp)
	twoKi = TWO_KI_DEF;	// 2 * integral gain (Ki)
	q0 = 1.0f;
	q1 = 0.0f;
	q2 = 0.0f;
	q3 = 0.0f;
	integralFBx = 0.0f;
	integralFBy = 0.0f;
	integralFBz = 0.0f;
	invSampleFreq = 1.0f / DEFAULT_SAMPLE_FREQ;
}

//-------------------------------------------------------------------------------------------
// Initializer

ZP_Error Mahony::begin(float sampleFrequency)
{
	ZP_Error result = ZP_ERROR_OK;

	if (sampleFrequency <= 0.0f) {
		result |= ZP_ERROR_INVALID_ARG;
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

//-------------------------------------------------------------------------------------------
// IMU algorithm update

ZP_Error Mahony::updateIMU(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
	if (!isInitialized) return ZP_ERROR_NOT_READY;
	ZP_Error result = ZP_ERROR_OK;
    float recipNorm = 0.0f;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

	// Compute feedback only if accelerometer measurement valid
	// (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		ZP_Error normStatus = invSqrt(ax * ax + ay * ay + az * az, recipNorm);
		result |= normStatus;

		if (normStatus == ZP_ERROR_OK) {
			ax *= recipNorm;
			ay *= recipNorm;
			az *= recipNorm;

			// Estimated direction of gravity
			halfvx = q0 * q2 - q1 * q3;
			halfvy = -(q0 * q1 + q2 * q3);
			halfvz = 0.5f - q0 * q0 - q3 * q3;

			// Error is sum of cross product between estimated
			// and measured direction of gravity
			halfex = (ay * halfvz - az * halfvy);
			halfey = (az * halfvx - ax * halfvz);
			halfez = (ax * halfvy - ay * halfvx);

			// Compute and apply integral feedback if enabled
			if(twoKi > 0.0f) {
				// integral error scaled by Ki
				integralFBx += twoKi * halfex * dt;
				integralFBy += twoKi * halfey * dt;
				integralFBz += twoKi * halfez * dt;
				gx += integralFBx;	// apply integral feedback
				gy += integralFBy;
				gz += integralFBz;
			} else {
				integralFBx = 0.0f;	// prevent integral windup
				integralFBy = 0.0f;
				integralFBz = 0.0f;
			}

			// Apply proportional feedback
			gx += twoKp * halfex;
			gy += twoKp * halfey;
			gz += twoKp * halfez;
		}
	}

	// Integrate rate of change of quaternion
	gx *= (0.5f * dt);		// pre-multiply common factors
	gy *= (0.5f * dt);
	gz *= (0.5f * dt);
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx);

	// Normalise quaternion
	result |= invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3, recipNorm);	
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;

	roll = atan2f(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2);
	pitch = asinf(-2.0f * (q1*q3 - q0*q2));
	yaw = atan2f(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3);

	return result;
}

//-------------------------------------------------------------------------------------------
// Attitude accessors

ZP_Error Mahony::getAttitude(Attitude_t& out_attitude)
{
	ZP_Error result = ZP_ERROR_OK;

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

ZP_Error Mahony::getAttitudeRadians(Attitude_t& outAttitude)
{
	ZP_Error result = ZP_ERROR_OK;

	if (!isInitialized) {
		result |= ZP_ERROR_NOT_READY;
	}

	if (result == ZP_ERROR_OK) {
		outAttitude.roll = roll;
		outAttitude.pitch = pitch;
		outAttitude.yaw = yaw;
	}
	return result;
}

//-------------------------------------------------------------------------------------------
// Fast inverse square-root

ZP_Error Mahony::invSqrt(float x, float &output)
{
	if (x <= 0) return ZP_ERROR_INVALID_ARG;

	float halfx = 0.5f * x;
	union { float f; long l; } i;
	i.f = x;
	i.l = 0x5f3759df - (i.l >> 1);
	float y = i.f;
	y = y * (1.5f - (halfx * y * y));
	y = y * (1.5f - (halfx * y * y));
	output = y;
	return ZP_ERROR_OK;
}

//============================================================================================
// END OF CODE
//============================================================================================
