#ifndef UV_SENSOR_H
#define UV_SENSOR_H

#include <stdint.h>

#include "stm32l4xx_hal.h"

HAL_StatusTypeDef VEML6070_Init(void);
HAL_StatusTypeDef VEML6070_Stop(void);
HAL_StatusTypeDef VEML6070_ReadUv(uint16_t* uv_raw);

#endif /* UV_SENSOR_H */
