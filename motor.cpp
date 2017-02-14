#include <Arduino.h>
#include "motor.h"

uint8_t motor_pin[4];

void motor_set_pin(uint8_t front_right_pin, uint8_t front_left_pin, uint8_t back_left_pin, uint8_t back_right_pin)
{
	motor_pin[MOTOR_FRONT_RIGHT] = front_right_pin;
	pinMode(motor_pin[MOTOR_FRONT_RIGHT], OUTPUT);
	analogWrite(motor_pin[MOTOR_FRONT_RIGHT], 0);

	motor_pin[MOTOR_FRONT_LEFT] = front_left_pin;
	pinMode(motor_pin[MOTOR_FRONT_LEFT], OUTPUT);
	analogWrite(motor_pin[MOTOR_FRONT_LEFT], 0);

	motor_pin[MOTOR_BACK_LEFT] = back_left_pin;
	pinMode(motor_pin[MOTOR_BACK_LEFT], OUTPUT);
	analogWrite(motor_pin[MOTOR_BACK_LEFT], 0);

	motor_pin[MOTOR_BACK_RIGHT] = back_right_pin;
	pinMode(motor_pin[MOTOR_BACK_RIGHT], OUTPUT);
	analogWrite(motor_pin[MOTOR_BACK_RIGHT], 0);
}

// Set motors speed in one time
void motor_set_speed(uint16_t front_right, uint16_t front_left, uint16_t back_left, uint16_t back_right)
{
	if (front_right > MOTOR_SPEED_MAX)
		front_right = MOTOR_SPEED_MAX;
	
	if (front_left > MOTOR_SPEED_MAX)
		front_left = MOTOR_SPEED_MAX;

	if (back_left > MOTOR_SPEED_MAX)
		back_left = MOTOR_SPEED_MAX;

	if (back_right > MOTOR_SPEED_MAX)
		back_right = MOTOR_SPEED_MAX;

	analogWrite(motor_pin[MOTOR_FRONT_RIGHT], 0);
	analogWrite(motor_pin[MOTOR_FRONT_LEFT], 0);
	analogWrite(motor_pin[MOTOR_BACK_LEFT], 0);
	analogWrite(motor_pin[MOTOR_BACK_RIGHT], 0);	
}
