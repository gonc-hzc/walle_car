#ifndef __ATTITUDE_H
#define __ATTITUDE_H

#include "bmi088.h"
#include "stm32f1xx_hal.h"

typedef struct
{
    int32_t x;
    int32_t y;
    int32_t z;
} Attitude_Vector3i;

typedef struct
{
    Attitude_Vector3i accel_mg;
    Attitude_Vector3i gyro_mdps;
    int32_t accel_roll_cdeg;
    int32_t accel_pitch_cdeg;
    int32_t mag_yaw_cdeg;
    int32_t roll_cdeg;
    int32_t pitch_cdeg;
    int32_t yaw_cdeg;
} Attitude_State;

int8_t Attitude_CalibrateGyro(BMI088_RawData *gyro_offset);
void Attitude_Init(Attitude_State *state);
void Attitude_Update(Attitude_State *state,
                     const BMI088_RawData *accel_raw,
                     const BMI088_RawData *gyro_raw,
                     const BMI088_RawData *gyro_offset,
                     uint32_t now_tick);
void Attitude_Update9Axis(Attitude_State *state,
                          const BMI088_RawData *accel_raw,
                          const BMI088_RawData *gyro_raw,
                          const BMI088_RawData *gyro_offset,
                          const Attitude_Vector3i *mag,
                          uint32_t now_tick);
int32_t Attitude_Abs32(int32_t value);

#endif
