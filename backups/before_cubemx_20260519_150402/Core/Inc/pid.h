#ifndef __PID_H
#define __PID_H

typedef struct
{
    float kp;
    float ki;
    float kd;

    float target;
    float actual;

    float error;
    float last_error;
    float integral;
    float integral_min;
    float integral_max;

    float output;
    float output_min;
    float output_max;
} PID_t;

void PID_Init(PID_t *pid, float kp, float ki, float kd,
              float output_min, float output_max,
              float integral_min, float integral_max);
float PID_Calc(PID_t *pid, float target, float actual);
void PID_Clear(PID_t *pid);

#endif
