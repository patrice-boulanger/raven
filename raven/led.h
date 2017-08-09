#ifndef _RAVEN_LEDS_H
#define _RAVEN_LEDS_H

#include "raven.h"

/*
 * Current LED sequence, clear the previous sequence. 
 * If 'seq' is zero, switch off all LEDs.
 */
void set_leds(const char *seq);

/*
 * Update LEDs status. To be called once in loop()
 */
void update_leds();

#endif // _RAVEN_LEDS_H
