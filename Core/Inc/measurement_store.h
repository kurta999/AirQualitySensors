#ifndef MEASUREMENT_STORE_H
#define MEASUREMENT_STORE_H

#include <stdint.h>

typedef struct
{
    float scd30_co2_ppm;
    float scd30_temperature_c;
    float scd30_relative_humidity_percent;
    float bme680_temperature_c;
    float bme680_relative_humidity_percent;
    float bme680_pressure_hpa;
    uint32_t bme680_gas_resistance_ohm;
    uint32_t bme680_timestamp_ms;
    uint16_t pm25_ug_m3;
    uint16_t pm10_ug_m3;
    uint32_t particle_timestamp_ms;
    uint16_t uv_raw;
    float red;
    float green;
    float blue;
    uint16_t color_temperature_k;
    uint16_t lux;
} MeasurementSnapshot;

void Measurements_UpdateScd30(float co2_ppm, float temperature_c,
                              float relative_humidity_percent);
void Measurements_UpdateBme680(float temperature_c, float relative_humidity_percent,
                               float pressure_hpa, uint32_t gas_resistance_ohm,
                               uint32_t timestamp_ms);
void Measurements_UpdateParticleFromIsr(uint16_t pm25_ug_m3, uint16_t pm10_ug_m3,
                                        uint32_t timestamp_ms);
void Measurements_UpdateUv(uint16_t uv_raw);
void Measurements_UpdateLight(float red, float green, float blue,
                              uint16_t color_temperature_k, uint16_t lux);
void Measurements_GetSnapshot(MeasurementSnapshot* snapshot);

#endif /* MEASUREMENT_STORE_H */
