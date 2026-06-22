#ifndef __BMI088_H
#define __BMI088_H

#include "stm32f1xx_hal.h"

typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} BMI088_RawData;

typedef struct
{
    float x;
    float y;
    float z;
} BMI088_Vector3f;

int8_t BMI088_Init(void);
int8_t BMI088_ReadChipId(uint8_t *accel_id, uint8_t *gyro_id);
int8_t BMI088_Read_ACC(int16_t *ax, int16_t *ay, int16_t *az);
int8_t BMI088_Read_GYRO(int16_t *gx, int16_t *gy, int16_t *gz);
int8_t BMI088_ReadRaw(BMI088_RawData *accel, BMI088_RawData *gyro);
int8_t BMI088_ReadScaled(BMI088_Vector3f *accel_g, BMI088_Vector3f *gyro_dps);
int8_t BMI088_GetLastResult(void);

#endif
