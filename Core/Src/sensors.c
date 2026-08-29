#include "sensors.h"

#include "cmsis_os.h"
#include "measurement_store.h"
#include "scd_sensor.h"
#include "uv_sensor.h"

void SensorsTask(void const* argument)
{
    (void)argument;

    while(VEML6070_Init() != HAL_OK)
    {
        osDelay(1000U);
    }
    while(Scd30_Init() != 0)
    {
        osDelay(1000U);
    }

    for(;;)
    {
        uint16_t uv_raw = 0U;
        if(VEML6070_ReadUv(&uv_raw) == HAL_OK)
        {
            Measurements_UpdateUv(uv_raw);
        }
        (void)Scd30_Read();
        osDelay(1000U);
    }
}
