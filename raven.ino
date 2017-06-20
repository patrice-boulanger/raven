#include <Arduino.h>

#include "raven.h"
#include "CPPM.h"

// Command as signed percents
typedef struct {
	int throttle;
	int yaw;
	int pitch;
	int roll;	
} cppm_cmd_pct_t;

// Command sensitivity
uint8_t sensitivity = MANUAL_MODE_PWM_SENSITIVITY_MAX;

/* ----- Motors management ----- */

void set_motor_pwm(uint8_t fr, uint8_t fl, uint8_t br, uint8_t bl)
{
	analogWrite(PIN_MOTOR_FR, fr);
	analogWrite(PIN_MOTOR_FL, fl);
	analogWrite(PIN_MOTOR_BR, br);
	analogWrite(PIN_MOTOR_BL, bl);
}

/*
 * Fully manual control of the drone, based only on commands received from user.
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
void set_motor_speed_manual(const cppm_cmd_pct_t *cmd)
{
	if (cmd->throttle == 0)
		return;
	
	// Commands PWM values
	int pwm_yaw, pwm_pitch, pwm_roll;
	
	// Motors PWM values
	int pwm_fr, pwm_fl, pwm_br, pwm_bl;

	// Compute PWM delta for each command
	pwm_yaw = (int) (MANUAL_MODE_PWM_DELTA_MAX * (float)(cmd->yaw) / 100.0);
	pwm_pitch = (int) (MANUAL_MODE_PWM_DELTA_MAX * (float)(cmd->pitch) / 100.0);
	pwm_roll = (int) (MANUAL_MODE_PWM_DELTA_MAX * (float)(cmd->roll) / 100.0);
	
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
#if ARDUINO >= 157
	Wire.setClock(400000UL); // Set I2C frequency to 400kHz
#else
	TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
#endif 

	// Set PIN mode
	pinMode(PIN_MPU_INT, INPUT);
	pinMode(PIN_CPPM_SIG, INPUT);
	pinMode(PIN_CPPM_PWR, OUTPUT);
	pinMode(PIN_MOTOR_FR, OUTPUT);
	pinMode(PIN_MOTOR_FL, OUTPUT);
	pinMode(PIN_MOTOR_BR, OUTPUT);
	pinMode(PIN_MOTOR_BL, OUTPUT);
	pinMode(PIN_LED_0, OUTPUT);
	pinMode(PIN_LED_1, OUTPUT);
	pinMode(PIN_BATTERY, INPUT);

	// LEDs on
	digitalWrite(PIN_LED_0, HIGH);
	digitalWrite(PIN_LED_1, HIGH);
	
	// Motor speed at 0
	analogWrite(PIN_MOTOR_FR, 0);
	analogWrite(PIN_MOTOR_FL, 0);
	analogWrite(PIN_MOTOR_BR, 0);
	analogWrite(PIN_MOTOR_BL, 0);

	// Enable & initialize CPPM receiver
	digitalWrite(PIN_CPPM_PWR, HIGH);
	CPPM.begin();
	delay(100);

	Serial.println("Ready");
}

void loop()
{
	cppm_cmd_pct_t cmd;

	memset(&cmd, 0, sizeof(cppm_cmd_pct_t));
	
	CPPM.cycle();
	if (CPPM.synchronized()) {
		// Translate CPPM values to (signed) percents
		int val;

		val = CPPM.read_us(CPPM_THRO);
		cmd.throttle = map(val, CPPM_MIN_VALUE, CPPM_MAX_VALUE, 0, 100);

		val = CPPM.read_us(CPPM_ELEV);
		cmd.pitch = map(val, CPPM_MIN_VALUE, CPPM_MAX_VALUE, -100, 100);

		val = CPPM.read_us(CPPM_RUDD);				   
		cmd.yaw = map(val, CPPM_MIN_VALUE, CPPM_MAX_VALUE, -100, 100);
		
		val = CPPM.read_us(CPPM_AILE);
		cmd.roll = map(val, CPPM_MIN_VALUE, CPPM_MAX_VALUE, -100, 100);

		Serial.print("throttle = ");
		Serial.print(cmd.throttle);
		Serial.print(" yaw = ");
		Serial.print(cmd.yaw);
		Serial.print(" pitch = ");
		Serial.print(cmd.pitch);
		Serial.print(" roll = ");
		Serial.println(cmd.roll);
		
	} else {
		Serial.println("CPPM not synced");
	}

	set_motor_speed_manual(&cmd);
}
