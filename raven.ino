#include <Wire.h>

#include "led.h"
#include "hmc5883l.h"
#include "mpu6050.h"
#include "bmp180.h"

/*
 * Global setup 
 */
void setup(void)
{
	Serial.begin(9600);

	LED_init();
	LED_set_sequence("_________OOOOO");
	
	// Initialize I2C bus
	Wire.begin();
	
	// Setup compas w/ 8x averaging, 15Hz measurement rate and gain of 5
	HMC5883L_init(0x70, 0xA0);
	// Setup accelerometer/gyroscope
	MPU6050_init();
	// Setup barometer w/ high precision
	BMP180_init(BMP180_RES_HIGH);

	delay(1000);
	LED_set_sequence(0);
}

/*
 * Main loop
 */
void loop(void)
{
	// Update sensors	
	HMC5883L_update();
	MPU6050_update();
	BMP180_update();

	// Update the LED
	LED_update();
	
	delay(30);
}



