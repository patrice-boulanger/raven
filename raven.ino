#include "raven.h"
#include "motors.h"

/* 
 * Use bolderflight SBUS library for Teensy 
 * https://github.com/bolderflight/SBUS.git
 */
#include "SBUS.h"

// Command as signed percents
typedef struct {
	float throttle;
	float yaw;
	float pitch;
	float roll;	
} cmd_pct_t;

// FRSky XSR receiver in SBUS mode on serial3
SBUS xsr(Serial3);

// SBUS channels & data
float channels[16];
uint8_t failSafe;
uint16_t lostFrames = 0;
	
cmd_pct_t cmd;

// Debugging
unsigned long last_time = 0;

/* ----- Flight controller ----- */

/*
 * Full manual control of the drone, based only on commands received from user.
 * Don't apply any compensation, free flight style !
 *
 * To compute PWM, use the following rules:
 * 
 *  pitch < 0 -> nose raises up -> front motors speed increases & back motors speed decreases
 *  pitch > 0 -> nose dips -> front motors speed decreases & back motors speed increases
 *   roll < 0 -> rolls to the left -> left motors speed decreases & right motors speed increases
 *   roll > 0 -> rolls to the right -> left motors speed increases & right motors speed decreases
 *    yaw < 0 -> turns to the left -> front-right/back-left motors speed decreases & front-left/back-right motors speed increases
 *    yaw > 0 -> turns to the right -> front-right/back-left motors speed increases & front-left/back-right motors speed decreases
 */
void set_motor_speed_manual(const cmd_pct_t *cmd)
{
	if (cmd->throttle == 0) {
		motors_set_speed(0, 0, 0, 0);
		return;
	}

	// Smooth the speed parameter using square of the throttle command
	float throttle2 = cmd->throttle * cmd->throttle;
	
	uint16_t throttle_speed = MOTOR_MIN_SPEED + throttle2 * MOTOR_DELTA_SPEED;

	/*
	 * decrease speed_delta while throttle speed increase ??
	 * 
	 */
	float speed_delta = MOTOR_SPEED_REACTIVTY * throttle_speed;

	float front_right_adjust  = - cmd->roll - cmd->pitch + cmd->yaw,
		front_left_adjust =   cmd->roll - cmd->pitch - cmd->yaw,
		back_left_adjust  =   cmd->roll + cmd->pitch + cmd->yaw,
		back_right_adjust = - cmd->roll + cmd->pitch - delta_yaw;

	int16_t front_right_speed = throttle_speed + (int16_t)(front_right_adjust * speed_delta),
		front_left_speed  = throttle_speed + (int16_t)(front_left_adjust  * speed_delta),
		back_left_speed   = throttle_speed + (int16_t)(back_left_adjust   * speed_delta),
		back_right_speed  = throttle_speed + (int16_t)(back_right_adjust  * speed_delta);

	// Check limits
	if (front_right_speed > MOTOR_MAX_SPEED) front_right_speed = MOTOR_MAX_SPEED;
	if (front_left_speed > MOTOR_MAX_SPEED)	  front_left_speed = MOTOR_MAX_SPEED;
	if (back_left_speed > MOTOR_MAX_SPEED)	   back_left_speed = MOTOR_MAX_SPEED;
	if (back_right_speed > MOTOR_MAX_SPEED)	  back_right_speed = MOTOR_MAX_SPEED;

	motors_set_speed(front_left_speed, front_right_speed, back_right_speed, back_left_speed);

	// DEBUG: print motors speeds every 5 seconds
	if (millis() - last_time > 5000) {
		Serial.print("FR: ");
		Serial.print(front_right_speed);
		Serial.print(" FL: ");
		Serial.print(front_left_speed);
		Serial.print(" BR: ");
		Serial.print(back_right_speed);
		Serial.print(" BL: ");
		Serial.println(back_left_speed);

		last_time = millis();
	}
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
	motors_initialize(PIN_MOTOR_FR, PIN_MOTOR_FL, PIN_MOTOR_BR, PIN_MOTOR_BL); 
	
	Serial.println("Ready");

	motors_set_speed(0, 0, 0, 0);
}

void loop()
{	
	memset(&cmd, 0, sizeof(cmd_pct_t));
	
	if (xsr.readCal(&channels[0], &failSafe, &lostFrames)) {
		cmd.throttle = (0.1 + channels[SBUS_CHANNEL_THROTTLE]) / 2.0; // 0 -> 1
		cmd.pitch = channels[SBUS_CHANNEL_PITCH]; // -1 -> 1
		cmd.yaw = channels[SBUS_CHANNEL_YAW];     // -1 -> 1
		cmd.roll = channels[SBUS_CHANNEL_ROLL];   // -1 -> 1

/*
		for(int i = 0; i < 16; i ++) {
			Serial.print("Ch");
			Serial.print(i);
			Serial.print(": ");
			Serial.print(channels[i]);
			Serial.println(" ");
		}
*/		

/*		Serial.print("throttle = ");
		Serial.print(cmd.throttle);
		Serial.print(" yaw = ");
		Serial.print(cmd.yaw);
		Serial.print(" pitch = ");
		Serial.print(cmd.pitch);
		Serial.print(" roll = ");
		Serial.println(cmd.roll);*/

	}
	
	set_motor_speed_manual(&cmd);
}
