#include <Arduino.h>
#include <Wire.h>

#include "led.h"
#include "mpu6050.h"

// Number of loop iterations for gyroscope calibration
#define CALIBRATION_ITER 1500

// acceleration
int16_t accl_x, accl_y, accl_z;
// gyroscope data
double gyro_calibration_x, gyro_calibration_y, gyro_calibration_z;
int16_t gyro_x, gyro_y, gyro_z;
// temperature
int16_t temp;

void MPU6050_init()
{
	Wire.beginTransmission(MPU6050_ADDR);
        Wire.write(0x6B);  // PWR_MGMT_1 register
	Wire.write(0);     // wakes up the module 
	Wire.endTransmission(true);

	Wire.beginTransmission(MPU6050_ADDR); 
	Wire.write(0x1B);  // GYRO_CONFIG register
	Wire.write(0x08);  // set 500dps full scale
	Wire.endTransmission();

	gyro_calibration_x = gyro_calibration_y = gyro_calibration_z = 0.0f;	
}

void MPU6050_calibrate(void)
{
	LED_set_sequence("_____xx");
	
	for(int i = 0; i < CALIBRATION_ITER; i ++) {
		MPU6050_read();
		
		gyro_calibration_x += gyro_x;
		gyro_calibration_y += gyro_y;
		gyro_calibration_z += gyro_z;

		delay(3);
		LED_update();
	}

	gyro_calibration_x /= CALIBRATION_ITER;
	gyro_calibration_y /= CALIBRATION_ITER;
	gyro_calibration_z /= CALIBRATION_ITER;
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

void MPU6050_update(void)
{
	MPU6050_read();
	
	// compensate w/ calibration data
	gyro_x -= gyro_calibration_x;
	gyro_y -= gyro_calibration_y;
	gyro_z -= gyro_calibration_z;
}

void MPU6050_get_angles(float *pitch, float *roll, float dt)
{
#warning check if axis match roll/pitch attitude
	
	// integrate the gyroscope data -> int(angularSpeed) = angle
	*pitch += ((float)gyro_x / GYRO_SENSITIVITY) * dt; // angle around the X-axis
	*roll -= ((float)gyro_y / GYRO_SENSITIVITY) * dt;  // angle around the Y-axis

	// compensate for drift with accelerometer data if !bullshit
	// sensitivity = -2 to 2 G at 16Bit -> 2G = 32768 && 0.5G = 8192
	int forceMagnitudeApprox = abs(accl_x) + abs(accl_y) + abs(accl_z);
	if (forceMagnitudeApprox > 8192 && forceMagnitudeApprox < 32768) {
		// turning around the X axis results in a vector on the Y-axis
		float pitchAcc = atan2f((float)accl_y, (float)accl_z) * 180 / M_PI;
		*pitch = *pitch * 0.98 + pitchAcc * 0.02;

		// turning around the Y axis results in a vector on the X-axis
		float rollAcc = atan2f((float)accl_x, (float)accl_z) * 180 / M_PI;
		*roll = *roll * 0.98 + rollAcc * 0.02;
	}
}
