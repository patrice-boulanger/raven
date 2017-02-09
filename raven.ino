#include <Wire.h>

#include "led.h"
#include "hmc5883l.h"
#include "mpu6050.h"
#include "bmp180.h"

// Compensated pitch/roll angles (in degrees)
float pitch_angle, roll_angle;
// main loop previous timer
unsigned long previous;

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
	// I2C clock speed to 400kHz
	TWBR = 12; 

	// Setup barometer w/ high precision
	BMP180_init(BMP180_RES_HIGH);
	// Setup compas w/ 8x averaging, 15Hz measurement rate and gain of 5
	HMC5883L_init(0x70, 0xA0);
	// Setup accelerometer/gyroscope & proceed to calibration
	MPU6050_init();
	MPU6050_calibrate();		

	// Iniitialize global variables for safety
	pitch_angle = roll_angle = 0.0f;
	previous = 0;
	
	// switch off the LED
 	LED_set_sequence(0);
	
	delay(500);
}

/*
 * Main loop
 */
void loop(void)
{
	unsigned long now = millis();
	
	// Update sensors	
	HMC5883L_update();
	MPU6050_update();
	BMP180_update();

	float dt = (now - previous) / 1000.0f;
	MPU6050_get_angles(&pitch_angle, &roll_angle, dt);
	
	// Update the LED
	LED_update();

	previous = now;

	Serial.print("pitch = ");
	Serial.print(pitch_angle);
	Serial.print(" roll = ");
	Serial.println(roll_angle);
}



