#ifndef _RAVEN_GIMBAL_H
#define _RAVEN_GIMBAL_H

#include "raven.h"
#include <Servo.h>

// Initialize the gimbal servo
void gimbal_init();
// Move gimbal to check it's working fine
void gimbal_dance();
// Set the pitch angle of the gimbal
void gimbal_set_pitch(int pitch);

#endif // _RAVEN_GIMBAL_H
