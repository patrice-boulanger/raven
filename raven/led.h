#ifndef _RAVEN_LEDS_H
#define _RAVEN_LEDS_H

#include "raven.h"

// Switch on LED
void led_on(uint8_t pin);

// Switch off LED
void led_off(uint8_t pin);

// Switch off all LEDS
void led_clear();

/*
 * Current LED sequence, clear the previous sequence. 
 * If 'seq' is zero, switch off all LEDs.
 */
void led_sequence(const char *seq);

/*
 * Update LEDs status. To be called once in loop()
 */
void led_update();

#endif // _RAVEN_LEDS_H
