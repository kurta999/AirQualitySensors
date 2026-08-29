#include "measurement_store.h"

#include <stddef.h>

#include "FreeRTOS.h"
#include "task.h"

static MeasurementSnapshot measurements;

void Measurements_UpdateScd30(float co2_ppm, float temperature_c,
                              float relative_humidity_percent)
{
    taskENTER_CRITICAL();
    measurements.scd30_co2_ppm = co2_ppm;
    measurements.scd30_temperature_c = temperature_c;
    measurements.scd30_relative_humidity_percent = relative_humidity_percent;
    taskEXIT_CRITICAL();
}

void Measurements_UpdateBme680(float temperature_c, float relative_humidity_percent,
                               float pressure_hpa, uint32_t gas_resistance_ohm,
                               uint32_t timestamp_ms)
{
    taskENTER_CRITICAL();
    measurements.bme680_temperature_c = temperature_c;
    measurements.bme680_relative_humidity_percent = relative_humidity_percent;
    measurements.bme680_pressure_hpa = pressure_hpa;
    measurements.bme680_gas_resistance_ohm = gas_resistance_ohm;
    measurements.bme680_timestamp_ms = timestamp_ms;
    taskEXIT_CRITICAL();
}

void Measurements_UpdateParticleFromIsr(uint16_t pm25_ug_m3, uint16_t pm10_ug_m3,
                                        uint32_t timestamp_ms)
{
    const UBaseType_t interrupt_state = taskENTER_CRITICAL_FROM_ISR();
    measurements.pm25_ug_m3 = pm25_ug_m3;
    measurements.pm10_ug_m3 = pm10_ug_m3;
    measurements.particle_timestamp_ms = timestamp_ms;
    taskEXIT_CRITICAL_FROM_ISR(interrupt_state);
}

void Measurements_UpdateUv(uint16_t uv_raw)
{
    taskENTER_CRITICAL();
    measurements.uv_raw = uv_raw;
    taskEXIT_CRITICAL();
}

void Measurements_UpdateLight(float red, float green, float blue,
                              uint16_t color_temperature_k, uint16_t lux)
{
    taskENTER_CRITICAL();
    measurements.red = red;
    measurements.green = green;
    measurements.blue = blue;
    measurements.color_temperature_k = color_temperature_k;
    measurements.lux = lux;
    taskEXIT_CRITICAL();
}

void Measurements_GetSnapshot(MeasurementSnapshot* snapshot)
{
    if(snapshot == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    *snapshot = measurements;
    taskEXIT_CRITICAL();
}
