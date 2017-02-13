#ifndef _RAVEN_PID_H
#define _RAVEN_PID_H

typedef struct {
	// PID coefficients
	float kp, ki, kd;
	// max value allowed for this PID
	float max;
	// sum of past errors
	float err_sum;
	// previous *error 
	float err_prev;
} pid_t;

float compute_pid(float target, float current, pid_t *pid);

#endif  // _RAVEN_PID_H
