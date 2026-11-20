#include "sensors.h"

#include "app_config.h"
#include "cmsis_os.h"
#include "honeywell_sensor.h"
#include "measurement_store.h"
#include "scd_sensor.h"
#include "sensor_retry.h"
#include "task_health.h"
#include "uv_sensor.h"

#define SENSORS_LOOP_PERIOD_MS 1000U

/*
 * Every sensor on this task owns its own init/retry state. A sensor that is
 * absent or unpowered is retried once per loop iteration and never blocks the
 * others, so partial hardware failure degrades gracefully instead of taking
 * the whole task down.
 */

#if APP_VEML6070_ENABLED
static void service_uv(SensorRetryState* state)
{
    if(SensorRetry_IsInitialized(state) == 0U)
    {
        if(VEML6070_Init() != HAL_OK)
        {
            return;
        }
        SensorRetry_ReportInitialized(state);
    }

    uint16_t uv_raw = 0U;
    if(VEML6070_ReadUv(&uv_raw) == HAL_OK)
    {
        SensorRetry_ReportSuccess(state);
        Measurements_UpdateUv(uv_raw, osKernelSysTick());
    }
    else if(SensorRetry_ReportFailure(state) != 0U)
    {
        Measurements_Invalidate(MEASUREMENT_SOURCE_UV);
    }
}
#endif

#if APP_SCD30_ENABLED
static void service_scd30(SensorRetryState* state)
{
    if(SensorRetry_IsInitialized(state) == 0U)
    {
        if(Scd30_Init() != 0)
        {
            return;
        }
        SensorRetry_ReportInitialized(state);
    }

    if(Scd30_Poll() == 0)
    {
        SensorRetry_ReportSuccess(state);
    }
    else if(SensorRetry_ReportFailure(state) != 0U)
    {
        Measurements_Invalidate(MEASUREMENT_SOURCE_SCD30);
    }
}
#endif

void SensorsTask(void const* argument)
{
    (void)argument;

#if APP_VEML6070_ENABLED
    SensorRetryState uv_state;
    SensorRetry_Reset(&uv_state);
#endif
#if APP_SCD30_ENABLED
    SensorRetryState scd30_state;
    SensorRetry_Reset(&scd30_state);
#endif

    for(;;)
    {
        TaskHealth_ReportAlive(TASK_HEALTH_SENSORS, osKernelSysTick());

#if APP_VEML6070_ENABLED
        service_uv(&uv_state);
#endif
#if APP_SCD30_ENABLED
        service_scd30(&scd30_state);
#endif
        /* The HPMA runs on UART4, not I2C1, but this is the loop that gives
         * it a periodic chance to recover from a stalled reception. */
#if APP_HPMA_ENABLED
        Honeywell_Poll(osKernelSysTick());
#endif

        osDelay(SENSORS_LOOP_PERIOD_MS);
    }
}
