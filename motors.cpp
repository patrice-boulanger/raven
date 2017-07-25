#include "raven.h"
#include "motors.h"

// ESC controllers
#include <Servo.h>

Servo esc_fr, esc_fl, esc_br, esc_bl;
int pulse_fr, pulse_fl, pulse_br, pulse_bl;

uint16_t pulse_interval = ESC_MAX_PULSE_WIDTH - ESC_MIN_PULSE_WIDTH;

void motors_initialize(uint8_t pin_fr, uint8_t pin_fl, uint8_t pin_br, uint8_t pin_bl)
{
	/*
	 * According to https://www.arduino.cc/en/Reference/ServoAttach, attach() set min. pulse width to 544 microseconds
	 * and max. pulse width to 2400 microseconds. Set these values to 1000/2000 microseconds to fit common ESC settings.
	 */ 
	esc_fr.attach(pin_fr, ESC_MIN_PULSE_WIDTH, ESC_MAX_PULSE_WIDTH);
	esc_fl.attach(pin_fl, ESC_MIN_PULSE_WIDTH, ESC_MAX_PULSE_WIDTH);
	esc_br.attach(pin_br, ESC_MIN_PULSE_WIDTH, ESC_MAX_PULSE_WIDTH);
	esc_bl.attach(pin_bl, ESC_MIN_PULSE_WIDTH, ESC_MAX_PULSE_WIDTH);

	// Stop all engines
	esc_fr.writeMicroseconds(ESC_MIN_PULSE_WIDTH);
	esc_fl.writeMicroseconds(ESC_MIN_PULSE_WIDTH);
	esc_br.writeMicroseconds(ESC_MIN_PULSE_WIDTH);
	esc_bl.writeMicroseconds(ESC_MIN_PULSE_WIDTH);
}

void motors_set_speed(float speed_fr, float speed_fl, float speed_br, float speed_bl)
{
	// Check bounds
	if (speed_fr < 0) speed_fr = 0;
	if (speed_fl < 0) speed_fl = 0;
	if (speed_br < 0) speed_br = 0;
	if (speed_bl < 0) speed_bl = 0;	

	if (speed_fr > 1.0) speed_fr = 1.0;
	if (speed_fl > 1.0) speed_fl = 1.0;
	if (speed_br > 1.0) speed_br = 1.0;
	if (speed_bl > 1.0) speed_bl = 1.0;	
	
	pulse_fr = ESC_MIN_PULSE_WIDTH + (int)(speed_fr * pulse_interval);
	pulse_fl = ESC_MIN_PULSE_WIDTH + (int)(speed_fl * pulse_interval);
	pulse_br = ESC_MIN_PULSE_WIDTH + (int)(speed_br * pulse_interval);
	pulse_bl = ESC_MIN_PULSE_WIDTH + (int)(speed_bl * pulse_interval);
	
	esc_fr.writeMicroseconds(pulse_fr);
	esc_fl.writeMicroseconds(pulse_fl);
	esc_br.writeMicroseconds(pulse_br);
	esc_bl.writeMicroseconds(pulse_bl);		
}

void motors_dance()
{
	motors_set_speed(50, 0, 0, 0);
	delay(500);
	motors_set_speed(0, 50, 0, 0);
	delay(500);
	motors_set_speed(0, 0, 50, 0);
	delay(500);
	motors_set_speed(0, 0, 0, 50);
	delay(500);
	motors_set_speed(0, 0, 0, 0);
}

