#ifndef _RAVEN_MPU6050_H
#define _RAVEN_MPU6050_H

// I2C module address
#define MPU6050_ADDR 0x68

#define GYRO_SENSITIVITY 65.536

/*
 * Initialization
 */
void MPU6050_init();

/*
 * Calibrate the gyroscope
 */
void MPU6050_calibrate(void);

/*
 * Update all data from this sensor
 */
void MPU6050_update(void);

/*
 * Computer pitch and roll angles (radians) using a complementary filter
 */
void MPU6050_get_angles(float *pitch, float *roll, float dt);

#endif // _RAVEN_MPU6050_H
