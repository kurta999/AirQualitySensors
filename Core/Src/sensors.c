#include "cmsis_os.h"
#include "device_specific.h"
#include "scd30/scd30.h"
#include "bme680/bme680.h"
#include "bme680/bme680_defs.h"
#include "main.h"
#include <string.h>

#include "light_sensor.h"
#include "scd_sensor.h"
#include "bosch_sensor.h"
#include "honeywell_sensor.h"
#include "uv_sensor.h"

extern I2C_HandleTypeDef hi2c1;

uint16_t uv_val;

void SensorsTask(void const * argument)
{
  /* USER CODE BEGIN SensorsTask */
  int16_t err;
  uint16_t interval_in_seconds = 2;

  VEML6070_Init();
  scd_init();

  /* Infinite loop */
    for(;;)
    {
        uv_val = VEML6070_ReadUV();

        scd_read_loop();
        osDelay(1000);
    }
  /* USER CODE END SensorsTask */
}


