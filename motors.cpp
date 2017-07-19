#include "motors.h"

// ESC controllers
#include <Servo.h>
Servo esc_fr, esc_fl, esc_br, esc_bl;

void motors_initialize(uint8_t pin_fr, uint8_t pin_fl, uint8_t pin_br, uint8_t pin_bl)
{
	esc_fr.attach(pin_fr);
	esc_fl.attach(pin_fl);
	esc_br.attach(pin_br);
	esc_bl.attach(pin_bl);

	esc_fr.write(MOTOR_MIN);
	esc_fl.write(MOTOR_MIN);
	esc_br.write(MOTOR_MIN);
	esc_bl.write(MOTOR_MIN);
}

void motors_set_speed(int16_t speed_fr, int16_t speed_fl, int16_t speed_br, int16_t speed_bl)
{
	if (speed_fr < MOTOR_MIN) speed_fr = MOTOR_MIN;
	if (speed_fl < MOTOR_MIN) speed_fl = MOTOR_MIN;
	if (speed_br < MOTOR_MIN) speed_br = MOTOR_MIN;
	if (speed_bl < MOTOR_MIN) speed_bl = MOTOR_MIN;

	if (speed_fr > MOTOR_MAX) speed_fr = MOTOR_MAX;
	if (speed_fl > MOTOR_MAX) speed_fl = MOTOR_MAX;
	if (speed_br > MOTOR_MAX) speed_br = MOTOR_MAX;
	if (speed_bl > MOTOR_MAX) speed_bl = MOTOR_MAX;

	esc_fr.write(speed_fr);
	esc_fl.write(speed_fl);
	esc_br.write(speed_br);
	esc_bl.write(speed_bl);		
}


