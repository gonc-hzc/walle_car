#include "bmi088.h"

#include "bmi08x.h"
#include "bmi088_port.h"

#define BMI088_ACCEL_RANGE_G        (24.0f)
#define BMI088_GYRO_RANGE_DPS       (2000.0f)
#define BMI088_INT16_FULL_SCALE     (32768.0f)

static struct bmi08_dev bmi088_dev;
static BMI088_PortSensor accel_intf = BMI088_PORT_ACCEL;
static BMI088_PortSensor gyro_intf = BMI088_PORT_GYRO;
static int8_t last_result;

static int8_t BMI088_Check(int8_t rslt)
{
    last_result = rslt;
    return rslt;
}

int8_t BMI088_Init(void)
{
    BMI088_Port_Init();
    HAL_Delay(100);

    bmi088_dev.intf = BMI08_SPI_INTF;
    bmi088_dev.variant = BMI088_VARIANT;
    bmi088_dev.intf_ptr_accel = &accel_intf;
    bmi088_dev.intf_ptr_gyro = &gyro_intf;
    bmi088_dev.read = BMI088_Port_Read;
    bmi088_dev.write = BMI088_Port_Write;
    bmi088_dev.delay_us = BMI088_Port_DelayUs;
    bmi088_dev.read_write_len = 32;

    int8_t rslt = bmi08xa_init(&bmi088_dev);
    if (rslt != BMI08_OK)
    {
        return BMI088_Check(rslt);
    }

    rslt = bmi08g_init(&bmi088_dev);
    if (rslt != BMI08_OK)
    {
        return BMI088_Check(rslt);
    }

    bmi088_dev.accel_cfg.power = BMI08_ACCEL_PM_ACTIVE;
    rslt = bmi08a_set_power_mode(&bmi088_dev);
    if (rslt != BMI08_OK)
    {
        return BMI088_Check(rslt);
    }

    bmi088_dev.accel_cfg.odr = BMI08_ACCEL_ODR_100_HZ;
    bmi088_dev.accel_cfg.bw = BMI08_ACCEL_BW_NORMAL;
    bmi088_dev.accel_cfg.range = BMI088_ACCEL_RANGE_24G;
    rslt = bmi08xa_set_meas_conf(&bmi088_dev);
    if (rslt != BMI08_OK)
    {
        return BMI088_Check(rslt);
    }

    bmi088_dev.gyro_cfg.power = BMI08_GYRO_PM_NORMAL;
    rslt = bmi08g_set_power_mode(&bmi088_dev);
    if (rslt != BMI08_OK)
    {
        return BMI088_Check(rslt);
    }

    bmi088_dev.gyro_cfg.odr = BMI08_GYRO_BW_32_ODR_100_HZ;
    bmi088_dev.gyro_cfg.range = BMI08_GYRO_RANGE_2000_DPS;
    rslt = bmi08g_set_meas_conf(&bmi088_dev);

    return BMI088_Check(rslt);
}

int8_t BMI088_ReadChipId(uint8_t *accel_id, uint8_t *gyro_id)
{
    if ((accel_id == NULL) || (gyro_id == NULL))
    {
        return BMI088_Check(BMI08_E_NULL_PTR);
    }

    uint8_t accel_rx[2] = {0};
    uint8_t gyro_rx = 0;

    BMI088_Port_Init();

    BMI08_INTF_RET_TYPE intf_rslt = BMI088_Port_Read(BMI08_REG_ACCEL_CHIP_ID | BMI08_SPI_RD_MASK,
                                                     accel_rx,
                                                     sizeof(accel_rx),
                                                     &accel_intf);
    if (intf_rslt != BMI08_INTF_RET_SUCCESS)
    {
        return BMI088_Check(BMI08_E_COM_FAIL);
    }

    intf_rslt = BMI088_Port_Read(BMI08_REG_ACCEL_CHIP_ID | BMI08_SPI_RD_MASK,
                                 accel_rx,
                                 sizeof(accel_rx),
                                 &accel_intf);
    if (intf_rslt != BMI08_INTF_RET_SUCCESS)
    {
        return BMI088_Check(BMI08_E_COM_FAIL);
    }

    intf_rslt = BMI088_Port_Read(BMI08_REG_GYRO_CHIP_ID | BMI08_SPI_RD_MASK,
                                 &gyro_rx,
                                 1,
                                 &gyro_intf);
    if (intf_rslt != BMI08_INTF_RET_SUCCESS)
    {
        return BMI088_Check(BMI08_E_COM_FAIL);
    }

    *accel_id = accel_rx[1];
    *gyro_id = gyro_rx;

    return BMI088_Check(BMI08_OK);
}

int8_t BMI088_Read_ACC(int16_t *ax, int16_t *ay, int16_t *az)
{
    if ((ax == NULL) || (ay == NULL) || (az == NULL))
    {
        return BMI088_Check(BMI08_E_NULL_PTR);
    }

    struct bmi08_sensor_data accel;
    int8_t rslt = bmi08a_get_data(&accel, &bmi088_dev);

    if (rslt == BMI08_OK)
    {
        *ax = accel.x;
        *ay = accel.y;
        *az = accel.z;
    }

    return BMI088_Check(rslt);
}

int8_t BMI088_Read_GYRO(int16_t *gx, int16_t *gy, int16_t *gz)
{
    if ((gx == NULL) || (gy == NULL) || (gz == NULL))
    {
        return BMI088_Check(BMI08_E_NULL_PTR);
    }

    struct bmi08_sensor_data gyro;
    int8_t rslt = bmi08g_get_data(&gyro, &bmi088_dev);

    if (rslt == BMI08_OK)
    {
        *gx = gyro.x;
        *gy = gyro.y;
        *gz = gyro.z;
    }

    return BMI088_Check(rslt);
}

int8_t BMI088_ReadRaw(BMI088_RawData *accel, BMI088_RawData *gyro)
{
    if ((accel == NULL) || (gyro == NULL))
    {
        return BMI088_Check(BMI08_E_NULL_PTR);
    }

    int8_t rslt = BMI088_Read_ACC(&accel->x, &accel->y, &accel->z);
    if (rslt != BMI08_OK)
    {
        return BMI088_Check(rslt);
    }

    rslt = BMI088_Read_GYRO(&gyro->x, &gyro->y, &gyro->z);

    return BMI088_Check(rslt);
}

int8_t BMI088_ReadScaled(BMI088_Vector3f *accel_g, BMI088_Vector3f *gyro_dps)
{
    if ((accel_g == NULL) || (gyro_dps == NULL))
    {
        return BMI088_Check(BMI08_E_NULL_PTR);
    }

    BMI088_RawData accel_raw;
    BMI088_RawData gyro_raw;
    int8_t rslt = BMI088_ReadRaw(&accel_raw, &gyro_raw);

    if (rslt == BMI08_OK)
    {
        accel_g->x = ((float)accel_raw.x * BMI088_ACCEL_RANGE_G) / BMI088_INT16_FULL_SCALE;
        accel_g->y = ((float)accel_raw.y * BMI088_ACCEL_RANGE_G) / BMI088_INT16_FULL_SCALE;
        accel_g->z = ((float)accel_raw.z * BMI088_ACCEL_RANGE_G) / BMI088_INT16_FULL_SCALE;

        gyro_dps->x = ((float)gyro_raw.x * BMI088_GYRO_RANGE_DPS) / BMI088_INT16_FULL_SCALE;
        gyro_dps->y = ((float)gyro_raw.y * BMI088_GYRO_RANGE_DPS) / BMI088_INT16_FULL_SCALE;
        gyro_dps->z = ((float)gyro_raw.z * BMI088_GYRO_RANGE_DPS) / BMI088_INT16_FULL_SCALE;
    }

    return BMI088_Check(rslt);
}

int8_t BMI088_GetLastResult(void)
{
    return last_result;
}
