#pragma once
#ifndef MADGWICK_HPP_
#define MADGWICK_HPP_

/* C/C++ Includes */
#include <stdint.h>

/* Eigen Includes */
#include "Eigen/Eigen"
#include "Eigen/Geometry"

class MadgwickFilter
{
public:		
	MadgwickFilter(const float sampleFrequency, const float beta_coeff);
	~MadgwickFilter();
	
	/** 
	 * Iterates the Madgwick algorithm once with the given input data.
	 * 
	 *@param accelerometer 
	 *	Acceleration measurements in vector form (x,y,z) with units m/s^2 
	 * 
	 *@param gyroscope
	 *	Gyroscope measurements in vector form (x,y,z) with units deg/s 
	 *		
	 *@param magnetometer
	 *	Magnetometer measurements in vector form (x,y,z) with units gauss */
	void update(Eigen::Vector3f accelerometer, Eigen::Vector3f gyroscope, Eigen::Vector3f magnetometer);
		
	/** Calculates the latest Euler angles in radians from the latest Quaternion vector */
	void getEulerRad(Eigen::Vector3f& eulerRad);
	
	/** Calculates the latest Euler angles in degrees from the latest Quaternion vector */
	void getEulerDeg(Eigen::Vector3f& eulerDeg);
	
	/** Returns the latest quaternion */
	void getQuaternion(Eigen::Quaternion<float>& quaternion);
	
private:	
	float sampleFreq, invSampleFreq;
	float beta;
	
	float invSqrt(float x);
	void update6DOF(Eigen::Vector3f accelerometer, Eigen::Vector3f gyroscope);
	void update9DOF(Eigen::Vector3f accelerometer, Eigen::Vector3f gyroscope, Eigen::Vector3f magnetometer);
	
	const int X = 0, Y = 1, Z = 2;
	
	float ax, ay, az;
	float gx, gy, gz;
	float mx, my, mz;
	float q0, q1, q2, q3;
	Eigen::Quaternion<float> outputQuaternion;	
};




/* REFERENCES: 
 * [1] S. O.H. Madgwick, "An Efficient Orientation Filter for Inertial and Inertial/Magnetic Sensor Arrays," 2010.
 *		http://x-io.co.uk/res/doc/madgwick_internal_report.pdf 
 *		
 * [2] */

#endif 