#include "measurement_format.h"

#include <inttypes.h>
#include <stdio.h>

/*
 * newlib-nano's snprintf() silently drops %f unless the build links
 * _printf_float, which this project deliberately does not do (it costs several
 * kilobytes of flash). Including the bundled tiny printf redirects snprintf()
 * to an implementation that formats floats unconditionally, and the host tests
 * link the very same implementation, so a passing test proves what the target
 * will actually emit.
 */
#include "newlib_fixes/printf.h"

/*
 * This record is a published wire format consumed by the backend
 * (https://github.com/kurta999/WindowsAddon). Fields must keep their exact
 * order and count -- the receiver extracts values positionally. Per-sensor
 * validity and staleness are tracked in MeasurementSnapshot and shown on the
 * LCD, but are deliberately NOT emitted here; adding fields would change what
 * the backend parses. Any change to this string is a protocol change: update
 * docs/protocol.md and the backend together.
 */
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
        measurement->timestamp_ms[MEASUREMENT_SOURCE_BME680],
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
