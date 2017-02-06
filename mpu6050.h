#ifndef _RAVEN_MPU6050_H
#define _RAVEN_MPU6050_H

// I2C module address
#define MPU6050_ADDR 0x68

/*
 * Initialization
 */
void MPU6050_init();

/*
 * Update all data from this sensor
 */
void MPU6050_update(void);

#endif // _RAVEN_MPU6050_H
