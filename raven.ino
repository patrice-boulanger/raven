#include "raven.h"
#include "motors.h"

/* 
 * Use bolderflight SBUS library for Teensy 
 * https://github.com/bolderflight/SBUS.git
 */
#include "SBUS.h"

// Command as signed percents
float throttle;
float yaw;
float pitch;
float roll;	
bool armed;
bool buzzer;

// FRSky XSR receiver in SBUS mode on serial3
SBUS xsr(Serial3);

// SBUS channels & data
float channels[16];
uint8_t failSafe;
uint16_t lostFrames = 0;
	
// Debugging
unsigned long last_time = 0;

/* ----- Flight controller ----- */

/*
 * Full manual control of the drone, based only on commands received from user.
 * Don't apply any compensation, free flight style !
 *
 * To compute each motor speed, use the following rules:
 * 
 *  pitch < 0 -> nose raises up -> front motors speed increases & back motors speed decreases
 *  pitch > 0 -> nose dips -> front motors speed decreases & back motors speed increases
 *   roll < 0 -> rolls left -> left motors speed decreases & right motors speed increases
 *   roll > 0 -> rolls right -> left motors speed increases & right motors speed decreases
 *    yaw < 0 -> rotate left -> front-right/back-left motors speed increases & front-left/back-right motors speed decreases
 *    yaw > 0 -> rotate right -> front-right/back-left motors speed decreases & front-left/back-right motors speed increases
 */
void set_motor_speed_manual()
{
	// Keep some room for motors adjustement
	if (throttle > 0.9)
		throttle = 0.9;

	float front_right_speed, front_left_speed, back_right_speed, back_left_speed;
	front_right_speed = front_left_speed = back_right_speed = back_left_speed = throttle;
	
	front_right_speed  += 0.15 * (- roll - pitch - yaw) / 3;
	back_left_speed    += 0.15 * (  roll + pitch - yaw) / 3;
	front_left_speed   += 0.15 * (  roll - pitch + yaw) / 3;
	back_right_speed   += 0.15 * (- roll + pitch + yaw) / 3;

	motors_set_speed(front_left_speed, front_right_speed, back_right_speed, back_left_speed);
}

/* ----- Main program ----- */

void setup()
{
	// Start serial console
	Serial.begin(9600);
	Serial.print("raven v");
	Serial.print(RAVEN_VERSION);
	Serial.println(" starting");
	
	// Initialize I2C bus
	Serial.println("Initializing I2C bus");
	Wire.begin();

	Serial.println("Initialize SBUS");
	xsr.begin();

	Serial.println("Initialize motors");
	motors_initialize(ESC_PIN_FR, ESC_PIN_FL, ESC_PIN_BR, ESC_PIN_BL); 
	
	Serial.println("Ready");

	motors_set_speed(0, 0, 0, 0);
}

void loop()
{	
	if (xsr.readCal(&channels[0], &failSafe, &lostFrames)) {
		if (failSafe) {
			// do something ...
			Serial.println("FAIL SAFE !!!");
			motors_set_speed(0,0,0,0);
			return;
		}

		buzzer = (channels[SBUS_CHANNEL_BUZZER] < 0.0);

		armed = (channels[SBUS_CHANNEL_ARMED] >= 0.0);
		if (!armed) {
			throttle = pitch = roll = yaw = 0.0;		
		} else {			
			throttle = (1.0 + channels[SBUS_CHANNEL_THROTTLE]) / 2.0; // 0 -> 1
			
			if (throttle > MOTORS_ARM_SPEED) {
				pitch = channels[SBUS_CHANNEL_PITCH]; // -1 -> 1
				roll = channels[SBUS_CHANNEL_ROLL];   // -1 -> 1
				yaw = channels[SBUS_CHANNEL_YAW];     // -1 -> 1
			} else
				throttle = MOTORS_ARM_SPEED;
		}
			
/*
		for(int i = 0; i < 16; i ++) {
			Serial.print("Ch");
			Serial.print(i);
			Serial.print(": ");
			Serial.print(channels[i]);
			Serial.print(" ");
		}

		Serial.println();
*/
		// DEBUG: print motors speeds every 5 seconds
		if (millis() - last_time > 2000) {
			Serial.print("Throttle: ");
			Serial.print(throttle);
			Serial.print(" Pitch: ");
			Serial.print(pitch);
			Serial.print(" Roll: ");
			Serial.print(roll);
			Serial.print(" Yaw: ");
			Serial.print(yaw);
			Serial.print(" Arm: ");
			Serial.print(armed);
			Serial.print(" Buzzer: ");
			Serial.println(buzzer);
			
			last_time = millis();
		}
	
		set_motor_speed_manual();
	} 
}
