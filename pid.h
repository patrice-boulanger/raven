#ifndef _RAVEN_PID_H
#define _RAVEN_PID_H

typedef struct {
	// PID coefficients
	float kp = 0, ki = 0, kd = 0;
	// max value allowed for this PID
	uint8_t max = 0;
	// sum of past errors
	float err_sum = 0;
	// previous error 
	float last_err = 0;
	// last computation time (ms)
	uint32_t last_time_ms = 0;
} pid_t;

float compute_pid(float setpoint, float measure, pid_t *pid);

#endif  // _RAVEN_PID_H
