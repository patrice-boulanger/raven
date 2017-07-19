#ifndef _RAVEN_MOTORS_H
#define _RAVEN_MOTORS_H

#include "raven.h"

void motors_initialize(uint8_t pin_fr, uint8_t pin_fl, uint8_t pin_br, uint8_t pin_bl);
void motors_set_speed(int16_t speed_fr, int16_t speed_fl, int16_t speed_br, int16_t speed_bl);

#endif // _RAVEN_MOTORS_H
