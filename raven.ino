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
	float speed = cmd->throttle * cmd->throttle;
	
	// Motors speeds, initialized according the throttle setpoint
	int speed_fr, speed_fl, speed_br, speed_bl;
	speed_fr = speed_fl = speed_bl = speed_br = MOTOR_MIN + (int)((MOTOR_MAX - MOTOR_MIN) * speed);       
	
	// Compute delta speeds for each command
	int delta_yaw, delta_pitch, delta_roll;
	delta_yaw = (int) (MOTOR_MANUAL_DELTA_MAX * cmd->yaw);
	delta_pitch = (int) (MOTOR_MANUAL_DELTA_MAX * cmd->pitch);
	delta_roll = (int) (MOTOR_MANUAL_DELTA_MAX * cmd->roll);
	
	// Adjust speeds for each motor
	speed_fr += - delta_roll - delta_pitch + delta_yaw;
	speed_fl +=   delta_roll - delta_pitch - delta_yaw;
	speed_bl +=   delta_roll + delta_pitch + delta_yaw;
	speed_br += - delta_roll + delta_pitch - delta_yaw;
		
	motors_set_speed(speed_fr, speed_fl, speed_br, speed_bl);

	// DEBUG: print motors speeds every 5 seconds
	if (millis() - last_time > 5000) {
		Serial.print("FR: ");
		Serial.print(speed_fr);
		Serial.print(" FL: ");
		Serial.print(speed_fl);
		Serial.print(" BR: ");
		Serial.print(speed_br);
		Serial.print(" BL: ");
		Serial.println(speed_bl);

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
