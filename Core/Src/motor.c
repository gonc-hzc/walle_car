#include "motor.h"
#include "stm32f103xb.h"
#include "stm32f1xx_hal_gpio.h"
#include "tim.h"

void Motor_Init(void)
{
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

    Motor_Left_Stop();
    Motor_Right_Stop();

    Motor_Left_SetPWM(0);
    Motor_Right_SetPWM(0);
}

static int Motor_LimitPWM(int pwm)
{
    if (pwm < 0) pwm = 0;
    if (pwm > MOTOR_PWM_MAX) pwm = MOTOR_PWM_MAX;
    return pwm;
}

void Motor_Left_SetPWM(int pwm)
{
    pwm = Motor_LimitPWM(pwm);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm);
}

void Motor_Right_SetPWM(int pwm)
{
    pwm = Motor_LimitPWM(pwm);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwm);
}

void Motor_Left_SetSignedPWM(int pwm)
{
    if (pwm > 0)
    {
        Motor_Left_Forward();
        Motor_Left_SetPWM(pwm);
    }
    else if (pwm < 0)
    {
        Motor_Left_Backward();
        Motor_Left_SetPWM(-pwm);
    }
    else
    {
        Motor_Left_SetPWM(0);
        Motor_Left_Stop();
    }
}

void Motor_Right_SetSignedPWM(int pwm)
{
    if (pwm > 0)
    {
        Motor_Right_Forward();
        Motor_Right_SetPWM(pwm);
    }
    else if (pwm < 0)
    {
        Motor_Right_Backward();
        Motor_Right_SetPWM(-pwm);
    }
    else
    {
        Motor_Right_SetPWM(0);
        Motor_Right_Stop();
    }
}

void Motor_Left_Forward(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
}

void Motor_Left_Backward(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
}

void Motor_Left_Stop(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
}

void Motor_Right_Forward(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
}

void Motor_Right_Backward(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
}

void Motor_Right_Stop(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
}
