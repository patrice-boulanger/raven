#ifndef _RAVEN_MPU6050_H
#define _RAVEN_MPU6050_H

// I2C module address
#define MPU6050_ADDR 0x68
// Full scale range
#define GYRO_FULL_SCALE_RANGE 0x08

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
 * Computer angles on x/Y axis (degrees) using a complementary filter
 */
void MPU6050_get_angles(float *compensated_angle_x, float *compensated_angle_y);

#endif // _RAVEN_MPU6050_H
