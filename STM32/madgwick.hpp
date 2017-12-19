#pragma once
#ifndef MADGWICK_HPP_
#define MADGWICK_HPP_

/* Eigen is used for any Matrix and/or Quaternion math */
#include "eigen/Eigen"
#include "eigen/Geometry"

template<typename Scalar_t>
	struct Vector3
	{
		Vector3()
		{
			x = 0.0;
			y = 0.0;
			z = 0.0;
		}
		
		Scalar_t x;
		Scalar_t y;
		Scalar_t z;
	};

template<typename Scalar_t>
	class MadgwickFilter
	{
	public:		
		MadgwickFilter(Scalar_t SAMPLE_RATE, Scalar_t BETA)
		{
			sample_rate = SAMPLE_RATE;
			beta = BETA;
			
			/* Initialize internal quaternions to unit vector */
			qEst.w() = 0.0; qDot.w() = 0.0;
			qEst.x() = 1.0; qDot.x() = 1.0;
			qEst.y() = 1.0; qDot.y() = 1.0;
			qEst.z() = 1.0; qDot.z() = 1.0;
		}
		
		~MadgwickFilter()
		{
			
		}
		
		/* Calculates the orientation estimate quaternion for the next time
		 * step given an input accelerometer, gyroscope, and magnetometer 
		 * measurement. It is assumed the matrices are in the format of [x, y, z].*/
		Eigen::Quaternion<Scalar_t> estimate(
			Vector3<Scalar_t> accelerometer,
			Vector3<Scalar_t> gyroscope,
			Vector3<Scalar_t> magnetometer);
		
		void toEuler(Scalar_t* theta, Scalar_t* phi, Scalar_t* psi);
		
	private:
		typedef Eigen::Quaternion<Scalar_t> Quaternion;
		
		Scalar_t sample_rate;
		Scalar_t beta;
		
		Vector3<Scalar_t> a, g, m;
		
		/* Records the quaternion estimate for each dt */
		Quaternion qEst;
		
		/* Records the rate of change quaternion for Gyroscope */
		Quaternion qDot;
		
		Quaternion calc_qDot(Quaternion prev_est);
	};

template<typename Scalar_t>
	Eigen::Quaternion<Scalar_t> MadgwickFilter<Scalar_t>::estimate(
		Vector3<Scalar_t> accelerometer,
		Vector3<Scalar_t> gyroscope,
		Vector3<Scalar_t> magnetometer)
	{
		/* Copy in the measured sensor data */
		a = accelerometer;
		g = gyroscope;
		m = magnetometer;
		
		/* Calculate the rate of change quaternion from Gyro */
		qDot = calc_qDot(qEst);
	}

template<typename Scalar_t>
	Eigen::Quaternion<Scalar_t> MadgwickFilter<Scalar_t>::calc_qDot(
	Eigen::Quaternion<Scalar_t> prev_est)
	{
		/* Calculates the result of (11) on pg.6 in [1] */
		Eigen::Quaternion<Scalar_t> result;
		Scalar_t q0 = prev_est.w();
		Scalar_t q1 = prev_est.x();
		Scalar_t q2 = prev_est.y();
		Scalar_t q3 = prev_est.z();
		
		result.w() = 0.5 * (-q1*g.x - q2*g.y - q3*g.z);
		result.x() = 0.5 * (q0*g.x + q2*g.z - q3*g.y);
		result.y() = 0.5 * (q0*g.y - q1*g.z + q3*g.x);
		result.z() = 0.5 * (q0*g.z + q1*g.y - q2*g.x);
		
		return result;
	}

template<typename Scalar_t>
	void MadgwickFilter<Scalar_t>::toEuler(Scalar_t* theta, Scalar_t* phi, Scalar_t* psi)
	{
		
	}

/* REFERENCES: 
 * [1] S. O.H. Madgwick, "An Efficient Orientation Filter for Inertial and Inertial/Magnetic Sensor Arrays," 2010.
 *		http://x-io.co.uk/res/doc/madgwick_internal_report.pdf 
 *		
 * [2] */

#endif 