#ifndef _RAVEN_STATE_H
#define _RAVEN_STATE_H

#include <Arduino.h>
#include "attitude.h"
#include "eeprom.h"
/*
 * jrowberg I2Cdev library for MPU6050, HMC5883L & BMP085/180
 * https://github.com/jrowberg/i2cdevlib
 */ 
#include "I2Cdev.h"
#include <Wire.h>
#include "MPU6050.h"
#include "HMC5883L.h"
#include "BMP085.h"

/*
 * Groups everything needed to compute the state of the drone
 */
typedef struct {
	// EEPROM configuration
	eeprom_config_t config;
	// Accelerometer/gyroscope
	MPU6050 mpu;
	// Magnetometer/compass
	HMC5883L compass;
	// Barometer
	BMP085 barometer;
	// SMA buffer
	sma_buffer_t sma;
	// Attitude data
	attitude_t attitude;
} state_t;

/*
 * Get raw values from sensors & add them to the SMA buffer
 */
void get_raw_value(state_t *state);

/*
 * Update the attitude based on the configuration & the elapsed time in microseconds
 */
void update_attitude(state_t *state, unsigned long micro);

#endif // _RAVEN_STATE_H
