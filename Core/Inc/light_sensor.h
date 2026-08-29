#ifndef LIGHT_SENSOR_H
#define LIGHT_SENSOR_H

#include "stm32l4xx_hal.h"

HAL_StatusTypeDef Tcs34725_Init(void);
HAL_StatusTypeDef Tcs34725_Read(void);

#endif /* LIGHT_SENSOR_H */
