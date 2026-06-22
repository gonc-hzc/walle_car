#include "bmi088_port.h"

#include "main.h"
#include "spi.h"

#define ACC_CS_LOW()   HAL_GPIO_WritePin(BMI088_ACC_CS2_GPIO_Port, BMI088_ACC_CS2_Pin, GPIO_PIN_RESET)
#define ACC_CS_HIGH()  HAL_GPIO_WritePin(BMI088_ACC_CS2_GPIO_Port, BMI088_ACC_CS2_Pin, GPIO_PIN_SET)

#define GYRO_CS_LOW()  HAL_GPIO_WritePin(BMI088_GYRO_CS1_GPIO_Port, BMI088_GYRO_CS1_Pin, GPIO_PIN_RESET)
#define GYRO_CS_HIGH() HAL_GPIO_WritePin(BMI088_GYRO_CS1_GPIO_Port, BMI088_GYRO_CS1_Pin, GPIO_PIN_SET)

#define BMI088_SPI_TIMEOUT_MS 100U

static void BMI088_Port_Select(void *intf_ptr)
{
    const BMI088_PortSensor *sensor = (const BMI088_PortSensor *)intf_ptr;

    if ((sensor != NULL) && (*sensor == BMI088_PORT_GYRO))
    {
        GYRO_CS_LOW();
    }
    else
    {
        ACC_CS_LOW();
    }
}

static void BMI088_Port_Deselect(void *intf_ptr)
{
    const BMI088_PortSensor *sensor = (const BMI088_PortSensor *)intf_ptr;

    if ((sensor != NULL) && (*sensor == BMI088_PORT_GYRO))
    {
        GYRO_CS_HIGH();
    }
    else
    {
        ACC_CS_HIGH();
    }
}

void BMI088_Port_Init(void)
{
    ACC_CS_HIGH();
    GYRO_CS_HIGH();
}

BMI08_INTF_RET_TYPE BMI088_Port_Read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    if ((reg_data == NULL) || (len == 0U))
    {
        return BMI08_E_NULL_PTR;
    }

    BMI088_Port_Select(intf_ptr);

    if (HAL_SPI_Transmit(&hspi2, &reg_addr, 1, BMI088_SPI_TIMEOUT_MS) != HAL_OK)
    {
        BMI088_Port_Deselect(intf_ptr);
        return BMI08_E_COM_FAIL;
    }

    for (uint32_t i = 0; i < len; i++)
    {
        uint8_t tx = 0x00;

        if (HAL_SPI_TransmitReceive(&hspi2, &tx, &reg_data[i], 1, BMI088_SPI_TIMEOUT_MS) != HAL_OK)
        {
            BMI088_Port_Deselect(intf_ptr);
            return BMI08_E_COM_FAIL;
        }
    }

    BMI088_Port_Deselect(intf_ptr);

    return BMI08_INTF_RET_SUCCESS;
}

BMI08_INTF_RET_TYPE BMI088_Port_Write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    if ((reg_data == NULL) || (len == 0U))
    {
        return BMI08_E_NULL_PTR;
    }

    BMI088_Port_Select(intf_ptr);

    if (HAL_SPI_Transmit(&hspi2, &reg_addr, 1, BMI088_SPI_TIMEOUT_MS) != HAL_OK)
    {
        BMI088_Port_Deselect(intf_ptr);
        return BMI08_E_COM_FAIL;
    }

    if (HAL_SPI_Transmit(&hspi2, (uint8_t *)reg_data, (uint16_t)len, BMI088_SPI_TIMEOUT_MS) != HAL_OK)
    {
        BMI088_Port_Deselect(intf_ptr);
        return BMI08_E_COM_FAIL;
    }

    BMI088_Port_Deselect(intf_ptr);

    return BMI08_INTF_RET_SUCCESS;
}

void BMI088_Port_DelayUs(uint32_t period, void *intf_ptr)
{
    (void)intf_ptr;

    HAL_Delay((period + 999U) / 1000U);
}
