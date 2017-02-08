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
	// I2C clock speed to 400kHz
	TWBR = 12; 
	
	// Setup compas w/ 8x averaging, 15Hz measurement rate and gain of 5
	HMC5883L_init(0x70, 0xA0);
	// Setup accelerometer/gyroscope
	MPU6050_init();
	// Setup barometer w/ high precision
	BMP180_init(BMP180_RES_HIGH);

	LED_set_sequence(0);

	delay(500);
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



