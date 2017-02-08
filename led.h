#ifndef _RAVEN_LED_H
#define _RAVEN_LED_H


/*
 * Initialize the LED module. The LED is switched off.
 */
void LED_init();

/*
 * Set the blinking sequence for the LED.
 *
 * The sequence is a string of character where '_' means switched off
 * and any other character means switch on. Each character stands for
 * 10ms. 
 */
void LED_set_sequence(const char *str);

/*
 * Update the status of the LED according to the last blinking sequence set.
 */
void LED_update();

#endif // _RAVEN_LED_H
