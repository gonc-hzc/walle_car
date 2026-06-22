#include "encoder.h"

volatile int32_t encoder_left_count = 0;
volatile int32_t encoder_right_count = 0;

void Encoder_Init(void)
{
    encoder_left_count = 0;
    encoder_right_count = 0;
}

int32_t Encoder_GetLeftCount(void)
{
    return encoder_left_count;
}

int32_t Encoder_GetRightCount(void)
{
    return encoder_right_count;
}

void Encoder_Clear(void)
{
    encoder_left_count = 0;
    encoder_right_count = 0;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_0)
    {
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_SET)
            encoder_left_count++;
        else
            encoder_left_count--;
    }

    if (GPIO_Pin == GPIO_PIN_1)
    {
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_SET)
            encoder_right_count--;
        else
            encoder_right_count++;
    }
}

int16_t Encoder_GetLeftSpeed(void)
{
    static int32_t last_count = 0;

    int32_t now_count = encoder_left_count;
    int16_t speed = now_count - last_count;

    last_count = now_count;

    return speed;
}

int16_t Encoder_GetRightSpeed(void)
{
    static int32_t last_count = 0;

    int32_t now_count = encoder_right_count;
    int16_t speed = now_count - last_count;

    last_count = now_count;

    return speed;
}