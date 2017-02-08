#include <Wire.h>
#include "mpu6050.h"

// acceleration
int16_t accl_x, accl_y, accl_z;
// gyroscope
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
	
}

void MPU6050_update(void)
{
	Wire.beginTransmission(MPU6050_ADDR);	
	Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
	Wire.endTransmission(false);
	
	Wire.requestFrom(MPU6050_ADDR, 14, true); // request a total of 14 registers
	accl_x = Wire.read() << 8 | Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
	accl_y = Wire.read() << 8 | Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
	accl_z = Wire.read() << 8 | Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
	temp   = Wire.read() << 8 | Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
	gyro_x = Wire.read() << 8 | Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
	gyro_y = Wire.read() << 8 | Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
	gyro_z = Wire.read() << 8 | Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

	temp = temp / 340.00 + 36.53; // convert to celsius
}
