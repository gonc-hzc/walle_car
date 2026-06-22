#ifndef __BLUETOOTH_H
#define __BLUETOOTH_H

#include "main.h"

void Bluetooth_Init(void);
void Bluetooth_Task(void);
void Bluetooth_RxCpltCallback(UART_HandleTypeDef *huart);
void Bluetooth_ErrorCallback(UART_HandleTypeDef *huart);

#endif
