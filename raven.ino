#include <Wire.h>

#include "led.h"
#include "hmc5883l.h"
#include "mpu6050.h"
#include "bmp180.h"
#include "rx.h"
#include "pid.h"
#include "motor.h"

// Motors PINS
const uint8_t PIN_FRONT_RIGHT = 5;
const uint8_t PIN_FRONT_LEFT = 6;
const uint8_t PIN_BACK_RIGHT = 9;
const uint8_t PIN_BACK_LEFT = 10;

#define THROTTLE_MANUAL_MAX 230

// !!! All angles values are expressed in degrees !!!

// Attitude based on compensated angles, the base referential is those of MPU6050
float x_measured_angle, y_measured_angle, z_measured_angle;
// Previous Z angle value & Z angular speed
float z_measured_angle_previous, z_real_angular_speed;
// Desired angles, computed from receiver orders & max values
float x_setpoint_angle, y_setpoint_angle, z_target_angular_speed;
// Maximal values allowed on X & Y axis 
float x_max_angle, y_max_angle;
// Maximal angular speed on Z axis (degrees / second)
float z_max_angular_speed;
// PID controllers for X/Y axis angles & Z angular speed
pid_t x_angle_pid, y_angle_pid, z_angular_speed_pid;
// Orders from PID controllers
float x_pid_compensation, y_pid_compensation, z_pid_compensation;

// loop timer 
uint32_t timer;

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

	// Set motors
	motor_set_pin(PIN_FRONT_RIGHT, PIN_FRONT_LEFT, PIN_BACK_LEFT, PIN_BACK_RIGHT);
	
	// Initialize I2C bus
	Wire.begin();
#if ARDUINO >= 157
	Wire.setClock(400000UL); // Set I2C frequency to 400kHz
#else
	TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
#endif 

	// Initialize SBUS receiver
	RX_init();
	
	// Setup barometer w/ high precision
	BMP180_init(BMP180_RES_HIGH);
	// Setup compas w/ 8x averaging, 15Hz measurement rate and gain of 5
	HMC5883L_init(0x70, 0xA0);
	// Setup accelerometer/gyroscope & calibrate
	MPU6050_init();
	MPU6050_calibrate();		

	// Iniitialize global variables for safety
	x_measured_angle = y_measured_angle = z_measured_angle = z_measured_angle_previous = 0.0f;
	z_real_angular_speed = 0.0f;
	x_setpoint_angle = y_setpoint_angle = z_target_angular_speed = 0.0f;
	
	x_max_angle = y_max_angle = 15.0;
	z_max_angular_speed = 30.0;

	x_pid_compensation = y_pid_compensation = z_pid_compensation = 0.0;

	
	// switch off the LED
 	LED_set_sequence("x_x__________");
	
	delay(500);

	timer = micros();
}

/*
 * Main loop
 */
void loop(void)
{
	float dt = (float)(micros() - timer) / 1000000.0;
	timer = micros();

	// Get commands from RX and translate to angles
	float throttle, yaw, pitch, roll;

	RX_get_command(&throttle, &yaw, &pitch, &roll);

#warning change coefficients

	x_setpoint_angle = x_max_angle * roll / 100.0;
	y_setpoint_angle = y_max_angle * pitch / 100.0;
	z_target_angular_speed = z_max_angular_speed * yaw / 100.0;
	
	// Update sensors & compute real angles values
	MPU6050_update();	
	HMC5883L_update();
	MPU6050_get_angles(&x_measured_angle, &y_measured_angle, dt);
	HMC5883L_get_heading_angle(x_measured_angle, y_measured_angle, &z_measured_angle);

	z_real_angular_speed = (z_measured_angle - z_measured_angle_previous) / dt;

	// Compute PID
	x_pid_compensation = compute_pid(x_setpoint_angle, x_measured_angle, &x_angle_pid);
	y_pid_compensation = compute_pid(y_setpoint_angle, y_measured_angle, &y_angle_pid);
	z_pid_compensation = compute_pid(z_target_angular_speed, z_real_angular_speed, &z_angular_speed_pid);

	// Compute motors
	uint16_t pwm_fr, pwm_fl, pwm_bl, pwm_br;

	// Initialize PWM according the throttle order
	pwm_fr = pwm_fl = pwm_bl = pwm_br = (uint16_t)(255.0 * throttle / 100.0);
	
	// keep some room for PID compensation
	if (pwm_fr > THROTTLE_MANUAL_MAX)
		pwm_fr = THROTTLE_MANUAL_MAX;
	
	if (pwm_fl > THROTTLE_MANUAL_MAX)
		pwm_fl = THROTTLE_MANUAL_MAX;

	if (pwm_bl > THROTTLE_MANUAL_MAX)
		pwm_bl = THROTTLE_MANUAL_MAX;

	if (pwm_br > THROTTLE_MANUAL_MAX)
		pwm_br = THROTTLE_MANUAL_MAX;

	// Adjust PWM for each motor
	pwm_fr += - y_pid_compensation + x_pid_compensation - z_pid_compensation;
	pwm_fl += - y_pid_compensation - x_pid_compensation + z_pid_compensation;
	pwm_bl +=   y_pid_compensation - x_pid_compensation - z_pid_compensation;
	pwm_br +=   y_pid_compensation + x_pid_compensation + z_pid_compensation;
	
	// Constraint the PWM to some limits
	if (pwm_fr > 255) pwm_fr = 255;
	if (pwm_fl > 255) pwm_fl = 255;
	if (pwm_bl > 255) pwm_bl = 255;
	if (pwm_br > 255) pwm_br = 255;

	motor_set_speed(pwm_fr, pwm_fl, pwm_bl, pwm_br);
	
	BMP180_update();
	
	// Update the LED
	LED_update();

	/*
	Serial.print("roll= ");
	Serial.print(x_measured_angle);
	Serial.print(" pitch = ");
	Serial.print(y_measured_angle);
	Serial.print(" heading = ");
	Serial.println(z_measured_angle);

	delay(10);
	*/
}



