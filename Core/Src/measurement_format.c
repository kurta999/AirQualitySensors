#include "measurement_format.h"

#include <inttypes.h>
#include <stdio.h>

int Measurements_Format(char* output, size_t output_capacity,
                        const MeasurementSnapshot* measurement)
{
    if(output == NULL || output_capacity == 0U || measurement == NULL)
    {
        return -1;
    }

    const int written = snprintf(
        output,
        output_capacity,
        "MEAS_DATA SCD30: %.4f,%.4f,%.4f BME680: %.4f,%.4f,%.4f,"
        "%" PRIu32 ",%" PRIu32 " HONEYWELL: %u,%u VEML6070: %u "
        "TCS: %.4f,%.4f,%.4f,%u,%u\n",
        (double)measurement->scd30_temperature_c,
        (double)measurement->scd30_relative_humidity_percent,
        (double)measurement->scd30_co2_ppm,
        (double)measurement->bme680_temperature_c,
        (double)measurement->bme680_relative_humidity_percent,
        (double)measurement->bme680_pressure_hpa,
        measurement->bme680_gas_resistance_ohm,
        measurement->bme680_timestamp_ms,
        (unsigned int)measurement->pm25_ug_m3,
        (unsigned int)measurement->pm10_ug_m3,
        (unsigned int)measurement->uv_raw,
        (double)measurement->red,
        (double)measurement->green,
        (double)measurement->blue,
        (unsigned int)measurement->color_temperature_k,
        (unsigned int)measurement->lux);

    if(written < 0 || (size_t)written >= output_capacity)
    {
        output[output_capacity - 1U] = '\0';
        return -1;
    }
    return written;
}
