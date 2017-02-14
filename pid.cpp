#include <Arduino.h>
#include "pid.h"


float compute_pid(float setpoint, float measure, pid_t *pid)
{
	uint32_t now = millis();
	float dt = (now - pid->last_time_ms) / 1000.0,
		err = setpoint - measure,
		res = 0;

	pid->err_sum += err * dt;
	float derr = (err - pid->last_err) / dt;

	res = pid->kp * err + pid->ki * pid->err_sum + pid->kd * derr;
	if (res > pid->max)
		res = pid->max;
	else if (res < -1 * pid->max)
		res = -1 * pid->max;
	
	pid->last_err = err;	
	pid->last_time_ms = now;

	return res;
}

/*
{
	float err = setpoint - measure, i_tmp = pid->ki * err, res = 0;

	// update error history
	pid->err_sum += i_tmp;
	if (pid->err_sum > pid->max)
		pid->err_sum = pid->max;
	else if (pid->err_sum < -1 * pid->max)
		pid->err_sum = -1 * pid->max;

	res = (pid->kp * err) + pid->err_sum + (pid->kd * (err - pid->err_prev));
	if (res > pid->max)
		res = pid->max;
	else if (res < -1 * pid->max)
		res = -1 * pid->max;
	
	pid->err_prev = err;

	return res;
}
*/
