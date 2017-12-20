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
#include "MadgwickAHRS.h"

#define SAMPLE_RATE 0.01 //sec
#define BETA 0.01
typedef float precisionType;

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
				   
	sensor_settings.scale.accel = A_SCALE_4G;
	sensor_settings.scale.mag = M_SCALE_2GS;
	sensor_settings.scale.gyro = G_SCALE_2000DPS;
				   
	sensor_settings.odr.accel = A_ODR_200;
	sensor_settings.odr.mag = M_ODR_100;
	sensor_settings.odr.gyro = G_ODR_190_BW_70;
	
	/* Create the new sensor object */
	LSM9DS0 sensor(sensor_settings);
	sensor.initialize();
	
	
	/* Initialize the Madgwick Filter */
	Eigen::Quaternion<precisionType> orientation;
	Vector3<precisionType> accel;
	Vector3<precisionType> gyro;
	Vector3<precisionType> mag;
	
	Madgwick MF;
	MF.begin(1000.0f);

	float roll, pitch, yaw;
	
	int counter = 0;

	for (;;)
	{
		if (counter == 100)
		{
			sensor.read(ALL);

			accel.x = sensor.accel_data_raw.x;
			accel.y = sensor.accel_data_raw.y;
			accel.z = sensor.accel_data_raw.z;

			gyro.x = sensor.gyro_data.x;
			gyro.y = sensor.gyro_data.y;
			gyro.z = sensor.gyro_data.z;

			mag.x = sensor.mag_data.x;
			mag.y = sensor.mag_data.y;
			mag.z = sensor.mag_data.z;

			counter = 0;
		}
		
		
		
		
		//orientation = AHRS.estimate(accel, gyro, mag);
		MF.update(gyro.x, gyro.y, gyro.z, accel.x, accel.y, accel.z, mag.x, mag.y, mag.z);

		roll = MF.getRoll();
		pitch = MF.getPitch();
		yaw = MF.getYaw();

		HAL_Delay(1);
		counter++;
	}
}
