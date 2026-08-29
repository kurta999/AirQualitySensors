#ifndef MEASUREMENT_STORE_H
#define MEASUREMENT_STORE_H

#include <stdint.h>

/**
 * Identifies one measurement source. Every source carries its own validity
 * bit and last-update timestamp so that a zeroed or stale reading can never
 * be mistaken for a fresh one.
 */
typedef enum
{
    MEASUREMENT_SOURCE_SCD30 = 0,
    MEASUREMENT_SOURCE_BME680,
    MEASUREMENT_SOURCE_PARTICLE,
    MEASUREMENT_SOURCE_UV,
    MEASUREMENT_SOURCE_LIGHT,
    MEASUREMENT_SOURCE_COUNT
} MeasurementSource;

/** Readings older than this are reported as stale by Measurements_IsFresh(). */
#define MEASUREMENT_DEFAULT_MAX_AGE_MS 15000U

typedef struct
{
    float scd30_co2_ppm;
    float scd30_temperature_c;
    float scd30_relative_humidity_percent;
    float bme680_temperature_c;
    float bme680_relative_humidity_percent;
    float bme680_pressure_hpa;
    uint32_t bme680_gas_resistance_ohm;
    uint16_t pm25_ug_m3;
    uint16_t pm10_ug_m3;
    uint16_t uv_raw;
    float red;
    float green;
    float blue;
    uint16_t color_temperature_k;
    uint16_t lux;

    /** Tick at which each source last produced a reading. */
    uint32_t timestamp_ms[MEASUREMENT_SOURCE_COUNT];
    /** One bit per MeasurementSource; set once a source has produced data. */
    uint32_t valid_mask;
    /** Tick at which this snapshot was taken; used to derive reading ages. */
    uint32_t snapshot_timestamp_ms;
} MeasurementSnapshot;

void Measurements_UpdateScd30(float co2_ppm, float temperature_c,
                              float relative_humidity_percent,
                              uint32_t timestamp_ms);
void Measurements_UpdateBme680(float temperature_c, float relative_humidity_percent,
                               float pressure_hpa, uint32_t gas_resistance_ohm,
                               uint32_t timestamp_ms);
void Measurements_UpdateParticleFromIsr(uint16_t pm25_ug_m3, uint16_t pm10_ug_m3,
                                        uint32_t timestamp_ms);
void Measurements_UpdateUv(uint16_t uv_raw, uint32_t timestamp_ms);
void Measurements_UpdateLight(float red, float green, float blue,
                              uint16_t color_temperature_k, uint16_t lux,
                              uint32_t timestamp_ms);

/**
 * Marks a source as having no usable reading, e.g. after it has been dropped
 * for repeated bus failures. The last values are left in place for debugging
 * but are no longer reported as valid.
 */
void Measurements_Invalidate(MeasurementSource source);

void Measurements_GetSnapshot(MeasurementSnapshot* snapshot);

/* --- Snapshot accessors; pure functions, usable on the host. ------------- */

/** Returns 1 when the source has produced at least one reading. */
uint8_t Measurements_IsValid(const MeasurementSnapshot* snapshot,
                             MeasurementSource source);

/**
 * Age of the source's reading in milliseconds relative to the moment the
 * snapshot was taken, or UINT32_MAX when the source has never been valid.
 */
uint32_t Measurements_AgeMs(const MeasurementSnapshot* snapshot,
                            MeasurementSource source);

/** Returns 1 when the source is valid and its reading is at most max_age_ms old. */
uint8_t Measurements_IsFresh(const MeasurementSnapshot* snapshot,
                             MeasurementSource source, uint32_t max_age_ms);

#endif /* MEASUREMENT_STORE_H */
