#include "gimbal.h"

// Pitch compensation servo
Servo gimbal;

void gimbal_init()
{
	gimbal.attach(GIMBAL_PIN);
	gimbal.write(GIMBAL_MIDDLE);
}

void gimbal_dance()
{
	gimbal.write(GIMBAL_MAX_ANGLE);
	delay(500);
	gimbal.write(GIMBAL_MIN_ANGLE);
	delay(500);
	gimbal.write(GIMBAL_MIDDLE);
}

void gimbal_set_pitch(int pitch)
{
	if (pitch < GIMBAL_MIN_ANGLE)
		pitch = GIMBAL_MIN_ANGLE;
	else if (pitch > GIMBAL_MAX_ANGLE)
		pitch = GIMBAL_MAX_ANGLE;
		
	gimbal.write(pitch);		
}

