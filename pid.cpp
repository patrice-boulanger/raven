#include "pid.h"

float compute_pid(float target, float current, pid_t *pid)
{
	float err = target - current, i_tmp = pid->ki * err, res = 0;

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
