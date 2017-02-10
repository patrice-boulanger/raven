#include <Arduino.h>
#include <Wire.h>

#include "led.h"
#include "mpu6050.h"

// Number of loop iterations for gyroscope calibration
#define CALIBRATION_ITER 1500

#define RAD2DEG 57.2957786f

// acceleration data
double accl_calibration_x, accl_calibration_y;
int16_t accl_x, accl_y, accl_z;
// gyroscope data
double gyro_calibration_x, gyro_calibration_y, gyro_calibration_z;
int16_t gyro_x, gyro_y, gyro_z;
// temperature
int16_t temp;
// timer 
uint32_t timer;

void MPU6050_init()
{
	Wire.beginTransmission(MPU6050_ADDR);
	Wire.write(0x6B);  // PWR_MGMT_1 register
	Wire.write(0);     // wakes up the module 
	Wire.endTransmission(true);

	Wire.beginTransmission(MPU6050_ADDR); 
	Wire.write(0x1B);  // GYRO_CONFIG register
	Wire.write(GYRO_FULL_SCALE_RANGE);  // set 500dps full scale
	Wire.endTransmission();

	accl_calibration_x = accl_calibration_y = 0.0f;
	gyro_calibration_x = gyro_calibration_y = gyro_calibration_z = 0.0f;
	
	//start timer
	timer = micros();
}

void MPU6050_read(void)
{
	Wire.beginTransmission(MPU6050_ADDR);	
	Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
	Wire.endTransmission(false);
	
	Wire.requestFrom(MPU6050_ADDR, 14, true); // request a total of 14 registers
	while(Wire.available() < 14)
		;
	
	accl_x = Wire.read() << 8 | Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
	accl_y = Wire.read() << 8 | Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
	accl_z = Wire.read() << 8 | Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
	temp   = Wire.read() << 8 | Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
	gyro_x = Wire.read() << 8 | Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
	gyro_y = Wire.read() << 8 | Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
	gyro_z = Wire.read() << 8 | Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

	temp = temp / 340.00 + 36.53; // convert to celsius  
}

void MPU6050_calibrate(void)
{
	LED_set_sequence("___xx");
	
	for(int i = 0; i < CALIBRATION_ITER; i ++) {
		MPU6050_read();

		accl_calibration_x += accl_x;
		accl_calibration_y += accl_y;
    
		gyro_calibration_x += gyro_x;
		gyro_calibration_y += gyro_y;
		gyro_calibration_z += gyro_z;

		delay(3);
		LED_update();
	}

	accl_calibration_x /= CALIBRATION_ITER;
	accl_calibration_y /= CALIBRATION_ITER;
	
	gyro_calibration_x /= CALIBRATION_ITER;
	gyro_calibration_y /= CALIBRATION_ITER;
	gyro_calibration_z /= CALIBRATION_ITER;
}

void MPU6050_update(void)
{
	MPU6050_read();
	
	// compensate w/ calibration data
	accl_x -= accl_calibration_x;
	accl_y -= accl_calibration_y;

	gyro_x -= gyro_calibration_x;
	gyro_y -= gyro_calibration_y;
	gyro_z -= gyro_calibration_z;
}

void MPU6050_get_angles(float *compensated_angle_x, float *compensated_angle_y)
{
	double dt = (double)(micros() - timer) / 1000000.0;
	timer = micros();

	// Compute the orientation of the accelerometer relative to the earth (convert from radians to degrees)
	// Use this data to correct any cumulative errors in the orientation that the gyroscope develops.
	double roll = atan2(accl_y, accl_z) * RAD2DEG;
	double pitch = atan2(-accl_x, accl_z) * RAD2DEG;
	
	double gyro_x_rate = gyro_x / 131.0;
	double gyro_y_rate = gyro_y / 131.0;
	
	//This filter calculates the angle based MOSTLY on integrating the angular velocity to an angular displacement.
	//dt, recall, is the time between gathering data from the MPU6050.  We'll pretend that the 
	//angular velocity has remained constant over the time dt, and multiply angular velocity by 
	//time to get displacement.
	//The filter then adds a small correcting factor from the accelerometer ("roll" or "pitch"), so the gyroscope knows which way is down. 
	*compensated_angle_x = 0.96 * (*compensated_angle_x + gyro_x_rate * dt) + 0.04 * roll; 
	*compensated_angle_y = 0.96 * (*compensated_angle_y + gyro_y_rate * dt) + 0.04 * pitch; 
}
