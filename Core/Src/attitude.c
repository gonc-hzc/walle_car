#include "attitude.h"

#include <math.h>

#define ATTITUDE_ACCEL_FS_MG        24000
#define ATTITUDE_GYRO_FS_MDPS       2000000
#define ATTITUDE_INT16_FULL_SCALE   32768
#define ATTITUDE_GYRO_CAL_SAMPLES   200
#define ATTITUDE_RAD_TO_CDEG        5729.577951f
#define ATTITUDE_MAG_YAW_ALPHA      0.98f
#define ATTITUDE_DEFAULT_DT_S       0.05f
#define ATTITUDE_DEG_TO_RAD         0.017453292f
#define ATTITUDE_KALMAN_Q_ANGLE     0.001f
#define ATTITUDE_KALMAN_Q_BIAS      0.003f
#define ATTITUDE_KALMAN_R_MEASURE   0.03f

typedef struct
{
    float angle;
    float bias;
    float p00;
    float p01;
    float p10;
    float p11;
} Attitude_Kalman1D;

static float roll_deg;
static float pitch_deg;
static float yaw_deg;
static uint32_t last_tick;
static uint8_t filter_ready;
static Attitude_Kalman1D roll_kalman;
static Attitude_Kalman1D pitch_kalman;

static int32_t Attitude_ScaleRawToMilli(int32_t raw, int32_t full_scale_milli)
{
    return (int32_t)(((int64_t)raw * full_scale_milli) / ATTITUDE_INT16_FULL_SCALE);
}

static int32_t Attitude_DegToCdeg(float degree)
{
    return (int32_t)(degree * 100.0f);
}

static float Attitude_WrapDeg(float degree)
{
    while (degree > 180.0f)
    {
        degree -= 360.0f;
    }

    while (degree < -180.0f)
    {
        degree += 360.0f;
    }

    return degree;
}

static void Attitude_KalmanInit(Attitude_Kalman1D *filter, float angle_deg)
{
    filter->angle = angle_deg;
    filter->bias = 0.0f;
    filter->p00 = 0.0f;
    filter->p01 = 0.0f;
    filter->p10 = 0.0f;
    filter->p11 = 0.0f;
}

static float Attitude_KalmanUpdate(Attitude_Kalman1D *filter,
                                   float measured_angle_deg,
                                   float measured_rate_dps,
                                   float dt_s)
{
    float rate = measured_rate_dps - filter->bias;
    filter->angle += dt_s * rate;

    filter->p00 += dt_s * ((dt_s * filter->p11) - filter->p01 - filter->p10 + ATTITUDE_KALMAN_Q_ANGLE);
    filter->p01 -= dt_s * filter->p11;
    filter->p10 -= dt_s * filter->p11;
    filter->p11 += ATTITUDE_KALMAN_Q_BIAS * dt_s;

    float innovation = measured_angle_deg - filter->angle;
    float innovation_cov = filter->p00 + ATTITUDE_KALMAN_R_MEASURE;
    float k0 = filter->p00 / innovation_cov;
    float k1 = filter->p10 / innovation_cov;

    filter->angle += k0 * innovation;
    filter->bias += k1 * innovation;

    float p00_temp = filter->p00;
    float p01_temp = filter->p01;

    filter->p00 -= k0 * p00_temp;
    filter->p01 -= k0 * p01_temp;
    filter->p10 -= k1 * p00_temp;
    filter->p11 -= k1 * p01_temp;

    return filter->angle;
}

static void Attitude_CalcAccelAngleCdeg(int32_t ax_mg,
                                        int32_t ay_mg,
                                        int32_t az_mg,
                                        int32_t *roll_cdeg,
                                        int32_t *pitch_cdeg)
{
    float ax = (float)ax_mg;
    float ay = (float)ay_mg;
    float az = (float)az_mg;

    float roll_rad = atan2f(ay, az);
    float pitch_rad = atan2f(-ax, sqrtf((ay * ay) + (az * az)));

    *roll_cdeg = (int32_t)(roll_rad * ATTITUDE_RAD_TO_CDEG);
    *pitch_cdeg = (int32_t)(pitch_rad * ATTITUDE_RAD_TO_CDEG);
}

static int32_t Attitude_CalcMagYawCdeg(const Attitude_Vector3i *mag,
                                       float roll_deg_in,
                                       float pitch_deg_in)
{
    float mx = (float)mag->x;
    float my = (float)mag->y;
    float mz = (float)mag->z;
    float roll_rad = roll_deg_in * ATTITUDE_DEG_TO_RAD;
    float pitch_rad = pitch_deg_in * ATTITUDE_DEG_TO_RAD;

    float cr = cosf(roll_rad);
    float sr = sinf(roll_rad);
    float cp = cosf(pitch_rad);
    float sp = sinf(pitch_rad);

    float mag_x = (mx * cp) + (mz * sp);
    float mag_y = (mx * sr * sp) + (my * cr) - (mz * sr * cp);
    float yaw_rad = atan2f(-mag_y, mag_x);

    return (int32_t)(yaw_rad * ATTITUDE_RAD_TO_CDEG);
}

int32_t Attitude_Abs32(int32_t value)
{
    return (value < 0) ? -value : value;
}

int8_t Attitude_CalibrateGyro(BMI088_RawData *gyro_offset)
{
    if (gyro_offset == NULL)
    {
        return -1;
    }

    int32_t sum_x = 0;
    int32_t sum_y = 0;
    int32_t sum_z = 0;
    uint16_t valid_samples = 0;

    for (uint16_t i = 0; i < ATTITUDE_GYRO_CAL_SAMPLES; i++)
    {
        int16_t gx = 0;
        int16_t gy = 0;
        int16_t gz = 0;

        if (BMI088_Read_GYRO(&gx, &gy, &gz) == 0)
        {
            sum_x += gx;
            sum_y += gy;
            sum_z += gz;
            valid_samples++;
        }

        HAL_Delay(2);
    }

    if (valid_samples == 0)
    {
        return -1;
    }

    gyro_offset->x = (int16_t)(sum_x / valid_samples);
    gyro_offset->y = (int16_t)(sum_y / valid_samples);
    gyro_offset->z = (int16_t)(sum_z / valid_samples);

    return 0;
}

void Attitude_Init(Attitude_State *state)
{
    if (state != NULL)
    {
        *state = (Attitude_State){0};
    }

    roll_deg = 0.0f;
    pitch_deg = 0.0f;
    yaw_deg = 0.0f;
    last_tick = HAL_GetTick();
    filter_ready = 0U;
    Attitude_KalmanInit(&roll_kalman, 0.0f);
    Attitude_KalmanInit(&pitch_kalman, 0.0f);
}

void Attitude_Update(Attitude_State *state,
                     const BMI088_RawData *accel_raw,
                     const BMI088_RawData *gyro_raw,
                     const BMI088_RawData *gyro_offset,
                     uint32_t now_tick)
{
    Attitude_Update9Axis(state, accel_raw, gyro_raw, gyro_offset, NULL, now_tick);
}

void Attitude_Update9Axis(Attitude_State *state,
                          const BMI088_RawData *accel_raw,
                          const BMI088_RawData *gyro_raw,
                          const BMI088_RawData *gyro_offset,
                          const Attitude_Vector3i *mag,
                          uint32_t now_tick)
{
    if ((state == NULL) || (accel_raw == NULL) || (gyro_raw == NULL) || (gyro_offset == NULL))
    {
        return;
    }

    state->accel_mg.x = Attitude_ScaleRawToMilli(accel_raw->x, ATTITUDE_ACCEL_FS_MG);
    state->accel_mg.y = Attitude_ScaleRawToMilli(accel_raw->y, ATTITUDE_ACCEL_FS_MG);
    state->accel_mg.z = Attitude_ScaleRawToMilli(accel_raw->z, ATTITUDE_ACCEL_FS_MG);

    int32_t gx_corr = gyro_raw->x - gyro_offset->x;
    int32_t gy_corr = gyro_raw->y - gyro_offset->y;
    int32_t gz_corr = gyro_raw->z - gyro_offset->z;

    state->gyro_mdps.x = Attitude_ScaleRawToMilli(gx_corr, ATTITUDE_GYRO_FS_MDPS);
    state->gyro_mdps.y = Attitude_ScaleRawToMilli(gy_corr, ATTITUDE_GYRO_FS_MDPS);
    state->gyro_mdps.z = Attitude_ScaleRawToMilli(gz_corr, ATTITUDE_GYRO_FS_MDPS);

    Attitude_CalcAccelAngleCdeg(state->accel_mg.x,
                                state->accel_mg.y,
                                state->accel_mg.z,
                                &state->accel_roll_cdeg,
                                &state->accel_pitch_cdeg);

    float dt_s = (float)(now_tick - last_tick) / 1000.0f;
    last_tick = now_tick;

    if ((dt_s <= 0.0f) || (dt_s > 0.5f))
    {
        dt_s = ATTITUDE_DEFAULT_DT_S;
    }

    float gyro_x_dps = (float)state->gyro_mdps.x / 1000.0f;
    float gyro_y_dps = (float)state->gyro_mdps.y / 1000.0f;
    float gyro_z_dps = (float)state->gyro_mdps.z / 1000.0f;

    if (filter_ready == 0U)
    {
        roll_deg = (float)state->accel_roll_cdeg / 100.0f;
        pitch_deg = (float)state->accel_pitch_cdeg / 100.0f;
        yaw_deg = 0.0f;
        Attitude_KalmanInit(&roll_kalman, roll_deg);
        Attitude_KalmanInit(&pitch_kalman, pitch_deg);
        filter_ready = 1U;
    }
    else
    {
        float accel_roll_deg = (float)state->accel_roll_cdeg / 100.0f;
        float accel_pitch_deg = (float)state->accel_pitch_cdeg / 100.0f;

        roll_deg = Attitude_KalmanUpdate(&roll_kalman, accel_roll_deg, gyro_x_dps, dt_s);
        pitch_deg = Attitude_KalmanUpdate(&pitch_kalman, accel_pitch_deg, gyro_y_dps, dt_s);
        yaw_deg += gyro_z_dps * dt_s;
    }

    if (mag != NULL)
    {
        state->mag_yaw_cdeg = Attitude_CalcMagYawCdeg(mag, roll_deg, pitch_deg);
        float mag_yaw_deg = (float)state->mag_yaw_cdeg / 100.0f;
        float yaw_error_deg = Attitude_WrapDeg(mag_yaw_deg - yaw_deg);

        yaw_deg += (1.0f - ATTITUDE_MAG_YAW_ALPHA) * yaw_error_deg;
        yaw_deg = Attitude_WrapDeg(yaw_deg);
    }
    else
    {
        state->mag_yaw_cdeg = Attitude_DegToCdeg(yaw_deg);
        yaw_deg = Attitude_WrapDeg(yaw_deg);
    }

    state->roll_cdeg = Attitude_DegToCdeg(roll_deg);
    state->pitch_cdeg = Attitude_DegToCdeg(pitch_deg);
    state->yaw_cdeg = Attitude_DegToCdeg(yaw_deg);
}
