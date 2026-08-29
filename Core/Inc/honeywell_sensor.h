#ifndef HONEYWELL_SENSOR_H
#define HONEYWELL_SENSOR_H

#include "stm32l4xx_hal.h"

HAL_StatusTypeDef Honeywell_Init(void);
void Honeywell_UartRecv(UART_HandleTypeDef* huart);

#endif /* HONEYWELL_SENSOR_H */
