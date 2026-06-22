#include "pid.h"

static float PID_Limit(float value, float min, float max)
{
    if (value > max) value = max;
    if (value < min) value = min;
    return value;
}

void PID_Init(PID_t *pid, float kp, float ki, float kd,
              float output_min, float output_max,
              float integral_min, float integral_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->target = 0;
    pid->actual = 0;

    pid->error = 0;
    pid->last_error = 0;
    pid->integral = 0;
    pid->integral_min = integral_min;
    pid->integral_max = integral_max;

    pid->output = 0;
    pid->output_min = output_min;
    pid->output_max = output_max;
}

float PID_Calc(PID_t *pid, float target, float actual)
{
    pid->target = target;
    pid->actual = actual;

    pid->error = pid->target - pid->actual;
    pid->integral += pid->error;
    pid->integral = PID_Limit(pid->integral, pid->integral_min, pid->integral_max);

    float derivative = pid->error - pid->last_error;

    pid->output = pid->kp * pid->error
                + pid->ki * pid->integral
                + pid->kd * derivative;

    pid->output = PID_Limit(pid->output, pid->output_min, pid->output_max);

    pid->last_error = pid->error;

    return pid->output;
}

void PID_Clear(PID_t *pid)
{
    pid->error = 0;
    pid->last_error = 0;
    pid->integral = 0;
    pid->output = 0;
}
