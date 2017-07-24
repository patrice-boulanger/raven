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

	esc_fr.write(MOTOR_MIN_SPEED);
	esc_fl.write(MOTOR_MIN_SPEED);
	esc_br.write(MOTOR_MIN_SPEED);
	esc_bl.write(MOTOR_MIN_SPEED);
}

void motors_set_speed(int16_t speed_fr, int16_t speed_fl, int16_t speed_br, int16_t speed_bl)
{
	esc_fr.write(speed_fr);
	esc_fl.write(speed_fl);
	esc_br.write(speed_br);
	esc_bl.write(speed_bl);		
}


