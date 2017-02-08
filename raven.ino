#include <Wire.h>

#include "led.h"
#include "hmc5883l.h"
#include "mpu6050.h"
#include "bmp180.h"

#ifdef RAVEN_DEBUG
// Statistics
uint32_t main_loop_iter = 0, main_loop_time = 0;
#endif

/*
 * Global setup 
 */
void setup(void)
{
	Serial.begin(9600);

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
	
	// switch off the LED
 	LED_set_sequence(0);
	
	delay(500);
}

/*
 * Main loop
 */
void loop(void)
{
#ifdef RAVEN_DEBUG
	unsigned long start = millis();
#endif
	// Update sensors	
	HMC5883L_update();
	MPU6050_update();
	BMP180_update();

	// Update the LED
	LED_update();

#ifdef RAVEN_DEBUG
	main_loop_time += millis() - start;
	main_loop_iter ++;

	if (main_loop_iter && main_loop_iter % 100 == 0) {
		double avg = main_loop_time / main_loop_iter;
		
		Serial.print("DEBUG: Main loop stats ");
		Serial.print(avg, 2);
		Serial.print(" ms avg. ");
		Serial.print(1/avg, 2);
		Serial.println(" iter/s");
	}
#endif
	
	delay(30);
}



