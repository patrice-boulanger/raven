#include "raven.h"
#include "motor.h"

Motor::Motor(uint8_t _pin, uint16_t _min_pulse, uint16_t _max_pulse) :
	pin(_pin), min_pulse(_min_pulse), max_pulse(_max_pulse), speed(0)
{
	/*
	 * According to https://www.arduino.cc/en/Reference/ServoAttach, attach() set min. pulse width to 544 microseconds
	 * and max. pulse width to 2400 microseconds. Set these values to 1000/2000 microseconds to fit common ESC settings.
	 */ 
	esc.attach(pin, min_pulse, max_pulse);
	esc.writeMicroseconds(min_pulse);	
}

Motor::~Motor()
{
	esc.writeMicroseconds(0);
}

void Motor::set_pulse(uint16_t pulse)
{
	if (pulse < min_pulse) pulse = min_pulse;
	if (pulse > max_pulse) pulse = max_pulse;
	esc.writeMicroseconds(pulse);

	speed = pulse;
}
	
uint16_t Motor::get_pulse()
{
	return speed;
}


