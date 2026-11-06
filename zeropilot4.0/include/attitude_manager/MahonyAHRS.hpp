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
#include <cmath>
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
	static ZP_Error invSqrt(float x, float &output);
	bool isInitialized = false;

//-------------------------------------------------------------------------------------------
// Function declarations

public:
	Mahony();
	// Initializer
	ZP_Error begin(float sampleFrequency);

	ZP_Error updateIMU(float gx, float gy, float gz, float ax, float ay, float az, float dt);

	ZP_Error getAttitude(Attitude_t& out_attitude);

	ZP_Error getAttitudeRadians(Attitude_t& outAttitude);
};