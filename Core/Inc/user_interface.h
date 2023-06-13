#ifndef USER_INTERFACE_H_
#define USER_INTERFACE_H_

#include "st7565/ST7565.h"

void UI_Init(void);

typedef struct
{
  float temp, hum;
  int co2, uv, pm25, pm10, lux, cct;
} MeasData_t;

#endif
