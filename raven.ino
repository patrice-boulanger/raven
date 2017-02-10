#include <Wire.h>

#include "led.h"
#include "hmc5883l.h"
#include "mpu6050.h"
#include "bmp180.h"

// Attitude based on compensated angles, the base referential is those of MPU6050
float x_angle, y_angle, z_angle;

/*
 * Global setup 
 */
void setup(void)
{
	Serial.begin(9600);
	Serial.println("raven v0.1");

	// Initialize onboard LED and switch it on
	LED_init();
 	LED_set_sequence("x");
	LED_update();

	// Initialize I2C bus
	Wire.begin();
#if ARDUINO >= 157
	Wire.setClock(400000UL); // Set I2C frequency to 400kHz
#else
	TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
#endif 

	// Setup barometer w/ high precision
	BMP180_init(BMP180_RES_HIGH);
	// Setup compas w/ 8x averaging, 15Hz measurement rate and gain of 5
	HMC5883L_init(0x70, 0xA0);
	// Setup accelerometer/gyroscope & calibrate
	MPU6050_init();
	MPU6050_calibrate();		

	// Iniitialize global variables for safety
	x_angle = y_angle = z_angle = 0.0f;
	
	// switch off the LED
 	LED_set_sequence("x_x__________");
	
	delay(500);
}

/*
 * Main loop
 */
void loop(void)
{
	// Update sensors
	MPU6050_update();	
	HMC5883L_update();
	MPU6050_get_angles(&x_angle, &y_angle);
	HMC5883L_get_heading_angle(x_angle, y_angle, &z_angle);

	BMP180_update();
	
	// Update the LED
	LED_update();

	Serial.print("roll= ");
	Serial.print(x_angle);
	Serial.print(" pitch = ");
	Serial.print(y_angle);
	Serial.print(" heading = ");
	Serial.println(z_angle);

	delay(10);
}



