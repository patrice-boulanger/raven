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
float z_measured_angle_previous, z_measured_angular_speed;
// Desired angles, computed from receiver orders & max values
float x_setpoint_angle, y_setpoint_angle, z_setpoint_angular_speed;
// Maximal values allowed on X & Y axis 
float x_max_angle, y_max_angle;
// Maximal angular speed on Z axis (degrees / second)
float z_max_angular_speed;
// PID controllers for X/Y axis angles & Z angular speed
pid_t x_angle_pid, y_angle_pid, z_angular_speed_pid;
// Orders from PID controllers
int x_pid_compensation, y_pid_compensation, z_pid_compensation;

// loop timer 
uint32_t timer;

/*
 * Global setup 
 */
void setup(void)
{
	Serial.begin(9600);
	Serial.println("Raven v0.1 initializing ... ");

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

	Serial.println("Calibrate gyroscope & accelerometer ...");
	MPU6050_calibrate();		

	// Iniitialize global variables for safety
	x_measured_angle = y_measured_angle = z_measured_angle = z_measured_angle_previous = 0.0f;
	z_measured_angular_speed = 0.0f;
	x_setpoint_angle = y_setpoint_angle = z_setpoint_angular_speed = 0.0f;
	
	x_max_angle = y_max_angle = 15.0;
	z_max_angular_speed = 30.0;

	x_pid_compensation = y_pid_compensation = z_pid_compensation = 0.0;

	x_angle_pid.kp = y_angle_pid.kp = 1.5;
	x_angle_pid.ki = y_angle_pid.ki = 0.035;
	x_angle_pid.kd = y_angle_pid.kd = 0.01;
	
	// Change LED sequence
 	LED_set_sequence("x_x__________");

	// Initialize the timer and pause for 10ms (dt != 0)
	timer = micros();
	delay(10);

	Serial.println("Initialize angular speed ...");
	
	// Do not start with z_measured_angle_previous equals to zero or the 
	// 1st measurement will mislead the Z angular speed PID controller
	float dt = (float)(micros() - timer) / 1000000.0;
	timer = micros();

	MPU6050_update();	
	HMC5883L_update(); 
	MPU6050_get_angles(&x_measured_angle, &y_measured_angle, dt);
	HMC5883L_get_heading_angle(x_measured_angle, y_measured_angle, &z_measured_angle);

	z_measured_angle_previous = z_measured_angle;

	Serial.println("Ready");
	
	delay(500);
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

	x_setpoint_angle = x_max_angle * roll / 100.0;
	y_setpoint_angle = y_max_angle * pitch / 100.0;
	z_setpoint_angular_speed = z_max_angular_speed * yaw / 100.0;
	
	// Update sensors & measure roll/pitch angles & yaw angular speed
	MPU6050_update();	
	HMC5883L_update();
	MPU6050_get_angles(&x_measured_angle, &y_measured_angle, dt);
	HMC5883L_get_heading_angle(x_measured_angle, y_measured_angle, &z_measured_angle);
	
	z_measured_angular_speed = (z_measured_angle - z_measured_angle_previous) / dt;

	throttle = 50;
	
	// Do not compensate anything if motors are stopped ...
	if (throttle > 0) {

		// Compute PID
		x_pid_compensation = compute_pid(x_setpoint_angle, x_measured_angle, &x_angle_pid);
		y_pid_compensation = compute_pid(y_setpoint_angle, y_measured_angle, &y_angle_pid);
		z_pid_compensation = compute_pid(z_setpoint_angular_speed, z_measured_angular_speed, &z_angular_speed_pid);

		// Compute motors
		uint16_t pwm_fr, pwm_fl, pwm_bl, pwm_br;

		// Initialize PWM according the throttle setpoint
		// Since THROTTLE_MANUAL_MAX is less than the max. PWM duty cycle,
		// we keep some room for PID compensation
		pwm_fr = pwm_fl = pwm_bl = pwm_br = (uint16_t)(THROTTLE_MANUAL_MAX * throttle / 100.0);
	
		// Adjust PWM for each motor
		// Due to the position of the board, the Y axis of the MPU6050 points to the
		// front direction and the X axis points to the right. Also, the roll angle
		// (taken on Y axis) increases when the drone rolls on the right (decreases 
		// when rolling on the left). The pitch angle (taken on the X axis) increases
		// when the drone raises the nose and decreases when the nose of the drone 
		// dips. 
		pwm_fr +=   x_pid_compensation - y_pid_compensation - z_pid_compensation;
		pwm_fl +=   x_pid_compensation + y_pid_compensation + z_pid_compensation;
		pwm_bl += - x_pid_compensation + y_pid_compensation - z_pid_compensation;
		pwm_br += - x_pid_compensation - y_pid_compensation + z_pid_compensation;
	
		// Constraint PWM freq
		if (pwm_fr > 255) pwm_fr = 255;
		if (pwm_fl > 255) pwm_fl = 255;
		if (pwm_bl > 255) pwm_bl = 255;
		if (pwm_br > 255) pwm_br = 255;

		//motor_set_speed(pwm_fr, pwm_fl, pwm_bl, pwm_br);

		Serial.print("FRONT-RIGHT: ");
		Serial.print(pwm_fr);
		Serial.print(" FRONT-LEFT: ");
		Serial.print(pwm_fl);
		Serial.print(" BACK-LEFT: ");
		Serial.print(pwm_bl);
		Serial.print(" BACK-RIGHT: ");
		Serial.println(pwm_br);
	}
	
	BMP180_update();
	
	// Update the LED
	LED_update();

	z_measured_angle_previous = z_measured_angle;
}



