/**
 * Pure accessors over a MeasurementSnapshot. Deliberately free of any RTOS or
 * HAL dependency so that the firmware and the host tests share one
 * implementation of the validity and staleness rules.
 */
#include <stddef.h>

#include "measurement_store.h"

uint8_t Measurements_IsValid(const MeasurementSnapshot* snapshot,
                             MeasurementSource source)
{
    if(snapshot == NULL || source >= MEASUREMENT_SOURCE_COUNT)
    {
        return 0U;
    }
    return (snapshot->valid_mask & ((uint32_t)1U << (uint32_t)source)) != 0U
        ? 1U
        : 0U;
}

uint32_t Measurements_AgeMs(const MeasurementSnapshot* snapshot,
                            MeasurementSource source)
{
    if(Measurements_IsValid(snapshot, source) == 0U)
    {
        return UINT32_MAX;
    }

    /* Unsigned subtraction is correct across the 32-bit tick wraparound. */
    return snapshot->snapshot_timestamp_ms - snapshot->timestamp_ms[source];
}

uint8_t Measurements_IsFresh(const MeasurementSnapshot* snapshot,
                             MeasurementSource source, uint32_t max_age_ms)
{
    return Measurements_AgeMs(snapshot, source) <= max_age_ms ? 1U : 0U;
}
