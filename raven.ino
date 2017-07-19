#include "raven.h"

/* 
 * Use bolderflight SBUS library for Teensy 
 * https://github.com/bolderflight/SBUS.git
 */
#include "SBUS.h"

// Command as signed percents
typedef struct {
	int throttle;
	int yaw;
	int pitch;
	int roll;	
} cmd_pct_t;

// Command sensitivity
uint8_t sensitivity = MANUAL_MODE_PWM_SENSITIVITY_MAX;

// FRSky XSR receiver in SBUS mode on serial3
SBUS xsr(Serial3);

// SBUS channels & data
float channels[16];
uint8_t failSafe;
uint16_t lostFrames = 0;
	
cmd_pct_t cmd;
	
/* ----- Motors management ----- */

void set_motor_pwm(uint8_t fr, uint8_t fl, uint8_t br, uint8_t bl)
{
	analogWrite(PIN_MOTOR_FR, fr);
	analogWrite(PIN_MOTOR_FL, fl);
	analogWrite(PIN_MOTOR_BR, br);
	analogWrite(PIN_MOTOR_BL, bl);
}

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
	if (cmd->throttle == 0)
		return;
	
	// Commands PWM values
	int pwm_yaw, pwm_pitch, pwm_roll;
	
	// Motors PWM values
	int pwm_fr, pwm_fl, pwm_br, pwm_bl;

	// Compute PWM delta for each command
	pwm_yaw = (int) (sensitivity * (float)(cmd->yaw) / 100.0);
	pwm_pitch = (int) (sensitivity * (float)(cmd->pitch) / 100.0);
	pwm_roll = (int) (sensitivity * (float)(cmd->roll) / 100.0);
	
	// Initialize PWM according the throttle setpoint
	pwm_fr = pwm_fl = pwm_bl = pwm_br = (int)(MANUAL_MODE_PWM_MAX * (float)(cmd->throttle) / 100.0);
	
	// Adjust PWM for each motor
	pwm_fr += - pwm_roll - pwm_pitch + pwm_yaw;
	pwm_fl +=   pwm_roll - pwm_pitch - pwm_yaw;
	pwm_bl +=   pwm_roll + pwm_pitch + pwm_yaw;
	pwm_br += - pwm_roll + pwm_pitch - pwm_yaw;
	
	// Constraint PWM freq
	if (pwm_fr > 255) pwm_fr = 255;
	if (pwm_fr < 0) pwm_fr = 0;
	
	if (pwm_fl > 255) pwm_fl = 255;
	if (pwm_fl < 0) pwm_fl = 0;
	
	if (pwm_br > 255) pwm_br = 255;
	if (pwm_br < 0) pwm_br = 0;

	if (pwm_bl > 255) pwm_bl = 255;
	if (pwm_bl < 0) pwm_bl = 0;
	
	// Set speeds
	set_motor_pwm(pwm_fr, pwm_fl, pwm_br, pwm_bl);
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

	// Set PIN mode
	//pinMode(PIN_MPU_INT, INPUT);
	pinMode(PIN_MOTOR_FR, OUTPUT);
	pinMode(PIN_MOTOR_FL, OUTPUT);
	pinMode(PIN_MOTOR_BR, OUTPUT);
	pinMode(PIN_MOTOR_BL, OUTPUT);
	
	// Motor speed at 0
	analogWrite(PIN_MOTOR_FR, 0);
	analogWrite(PIN_MOTOR_FL, 0);
	analogWrite(PIN_MOTOR_BR, 0);
	analogWrite(PIN_MOTOR_BL, 0);

	Serial.println("Initialize SBUS");
	xsr.begin();

	Serial.println("Ready");
}

void loop()
{	
	memset(&cmd, 0, sizeof(cmd_pct_t));
	
	if (xsr.readCal(&channels[0], &failSafe, &lostFrames)) {
/*
		for(int i = 0; i < 16; i ++) {
			Serial.print("Ch");
			Serial.print(i);
			Serial.print(": ");
			Serial.print(channels[i]);
			Serial.println(" ");
		}
*/		
		// Translate SBUS values to (signed) percents
		cmd.throttle = (int)((100 + 100.0 * channels[SBUS_CHANNEL_THROTTLE]) / 2);
		cmd.pitch = (int)(100.0 * channels[SBUS_CHANNEL_PITCH]);
		cmd.yaw = (int)(100.0 * channels[SBUS_CHANNEL_YAW]);
		cmd.roll = (int)(100.0 * channels[SBUS_CHANNEL_ROLL]);

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
