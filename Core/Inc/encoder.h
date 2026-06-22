#ifndef __ENCODER_H
#define __ENCODER_H

#include "main.h"

extern volatile int32_t encoder_left_count;
extern volatile int32_t encoder_right_count;

void Encoder_Init(void);
int32_t Encoder_GetLeftCount(void);
int32_t Encoder_GetRightCount(void);
void Encoder_Clear(void);

int16_t Encoder_GetLeftSpeed(void);
int16_t Encoder_GetRightSpeed(void);

#endif