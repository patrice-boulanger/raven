#ifndef _RAVEN_MOTORS_H
#define _RAVEN_MOTORS_H

#include "raven.h"

/*
 * Initialize motors
 */
void motors_initialize(uint8_t pin_fr, uint8_t pin_fl, uint8_t pin_br, uint8_t pin_bl);

/*
 * Set speeds for each motor, between 0 & 100 %.
 */
void motors_set_speed(float speed_fr, float speed_fl, float speed_br, float speed_bl);

/*
 * Move each motor for 0.5s
 */
void motors_dance(void);

#endif // _RAVEN_MOTORS_H
