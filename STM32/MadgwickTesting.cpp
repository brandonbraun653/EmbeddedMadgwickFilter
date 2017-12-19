#include <stm32f7xx_hal.h>
#include <stm32_hal_legacy.h>

/* Custom interface to all the STM32 peripherals. Can be found here:
 * https://bitbucket.org/codex653/thor_stm32f767zit 
 * 
 * Note that this library is NOT required for the actual filter to work. 
 * It is simply used here for testing on my actual embedded system. Feel
 * free to use it or not. */
#include "thor.h"

/* Custom interface to the LSM9DS0 9-DOF IMU.
 * Replace with your own sensor library/IC combination. */
#include "stm32f7_lsm9ds0.h"

/* Necessary file for the filter */
#include "madgwick.hpp"

#define SAMPLE_RATE 0.1 //sec
#define BETA 0.01
typedef double precisionType;

int main(void)
{
	HAL_Init();
	ThorSystemClockConfig();
	
	/* Sensor Pinouts for SPI Use: 
	VIN: 3-5V input
	SCL:	CLK
	SDA:	MOSI
	CSG:	Gyro CS
	SDOG:	Gyro MISO
	CSXM:	Accelerometer & Magnetometer CS
	SDOXM:	Accelerometer & Magnetometer MISO


	Let's use SPI3 on the Nucleo Board and see how it works. CN11
	CLK:  Yellow Cable
	MISO: Blue Cable
	MOSI: Brown Cable
	CSG:  Orange Cable -> PF6
	CSXM: White Cable  -> PF7
	*/
	LSM9DS0_Settings sensor_settings;

	/* Select the protocol used */
	sensor_settings.interfaceMode = MODE_SPI;

	/* Fill in the Accelerometer and Mag details */
	sensor_settings.xmAddress = 0x1D;
	sensor_settings.xmCSPin = PIN_7;
	sensor_settings.xmCSPort = GPIOF;
				   
	sensor_settings.gAddress = 0x6B; 
	sensor_settings.gCSPin = PIN_6;
	sensor_settings.gCSPort = GPIOF;
				   
	sensor_settings.scale.accel = A_SCALE_2G;
	sensor_settings.scale.mag = M_SCALE_2GS;
	sensor_settings.scale.gyro = G_SCALE_245DPS;
				   
	sensor_settings.odr.accel = A_ODR_50;
	sensor_settings.odr.mag = M_ODR_50;
	sensor_settings.odr.gyro = G_ODR_190_BW_125;
	
	/* Create the new sensor object */
	LSM9DS0 sensor(sensor_settings);
	sensor.initialize();
	
	
	/* Initialize the Madgwick Filter */
	MadgwickFilter<precisionType> AHRS(SAMPLE_RATE, BETA);
	Eigen::Quaternion<precisionType> orientation;
	Vector3<precisionType> accel;
	Vector3<precisionType> gyro;
	Vector3<precisionType> mag;
	
	for (;;)
	{
		sensor.read(ALL);
		
		accel.x = sensor.accel_data.x;
		accel.y = sensor.accel_data.y;
		accel.z = sensor.accel_data.z;
		
		gyro.x = sensor.gyro_data.x;
		gyro.y = sensor.gyro_data.y;
		gyro.z = sensor.gyro_data.z;
		
		mag.x = sensor.mag_data.x;
		mag.y = sensor.mag_data.y;
		mag.z = sensor.mag_data.z;
		
		orientation = AHRS.estimate(accel, gyro, mag);
		
		HAL_Delay((uint32_t)(SAMPLE_RATE*1000.0));
	}
}
