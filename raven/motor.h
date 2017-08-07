#ifndef _RAVEN_MOTORS_H
#define _RAVEN_MOTORS_H

#include <Servo.h>
#include "raven.h"

class Motor
{
public:
	Motor(uint8_t pin, uint16_t min_pulse = ESC_PULSE_MIN_WIDTH, uint16_t max_pulse = ESC_PULSE_MAX_WIDTH);
	~Motor();

	/*
	 * Set the speed of the motor
	 */
	void set_pulse(uint16_t pulse);

	uint16_t get_pulse();

protected:
	Servo esc;
	
	uint8_t pin;
	uint16_t min_pulse, max_pulse;
	uint16_t speed;
};

#endif // _RAVEN_MOTORS_H
