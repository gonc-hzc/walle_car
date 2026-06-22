#ifndef __BMI088_PORT_H
#define __BMI088_PORT_H

#include "bmi08_defs.h"
#include "stm32f1xx_hal.h"

typedef enum
{
    BMI088_PORT_ACCEL = 0,
    BMI088_PORT_GYRO
} BMI088_PortSensor;

BMI08_INTF_RET_TYPE BMI088_Port_Read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
BMI08_INTF_RET_TYPE BMI088_Port_Write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);
void BMI088_Port_DelayUs(uint32_t period, void *intf_ptr);
void BMI088_Port_Init(void);

#endif
