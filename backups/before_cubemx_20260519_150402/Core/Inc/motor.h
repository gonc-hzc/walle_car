#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"

#define MOTOR_PWM_MAX 999

void Motor_Init(void);

void Motor_Left_SetPWM(int pwm);
void Motor_Right_SetPWM(int pwm);
void Motor_Left_SetSignedPWM(int pwm);
void Motor_Right_SetSignedPWM(int pwm);

void Motor_Left_Forward(void);
void Motor_Left_Backward(void);
void Motor_Left_Stop(void);

void Motor_Right_Forward(void);
void Motor_Right_Backward(void);
void Motor_Right_Stop(void);

#endif
