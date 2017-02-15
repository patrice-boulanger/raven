#include <Arduino.h>
#include "pid.h"

float compute_pid(float setpoint, float measure, pid_t *pid)
{
	uint32_t now = micros();
	float dt = (now - pid->last_time_ms) / 1000000.0,
		err = setpoint - measure;
	int res = 0;

	float err_sum = (err + pid->last_err) * dt / 2, 
		derr = (err - pid->last_err) / dt;

	res = pid->kp * err + pid->ki * err_sum + pid->kd * derr;
	if (pid->max > 0) {
		if (res > pid->max)
			res = pid->max;
		else if (res < -1 * pid->max)
			res = -1 * pid->max;
	}
	
	pid->last_err = err;	
	pid->last_time_ms = now;

	return res;
}
/*
{
	float err = setpoint - measure, i_tmp = pid->ki * err;
	int res = 0;

	Serial.print("err = ");
	Serial.print(err);

	// update error history
	pid->err_sum += i_tmp;
	if (pid->max > 0) {
		if (pid->err_sum > pid->max)
			pid->err_sum = pid->max;
		else if (pid->err_sum < -1 * pid->max)
			pid->err_sum = -1 * pid->max;
	}
	
	Serial.print(" Sum = ");
	Serial.print(pid->err_sum);
	Serial.print(" dErr = ");
	Serial.println(err - pid->last_err);

	res = (pid->kp * err) + pid->err_sum + (pid->kd * (err - pid->last_err));
	if (pid->max > 0) {
		if (res > pid->max)
			res = pid->max;
		else if (res < -1 * pid->max)
			res = -1 * pid->max;
	}
	
	pid->last_err = err;

	return res;
}
*/
