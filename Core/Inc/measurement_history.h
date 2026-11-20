#ifndef MEASUREMENT_HISTORY_H
#define MEASUREMENT_HISTORY_H
#include "measurement_store.h"
#include <stddef.h>
#include <stdint.h>

#define HISTORY_CAPACITY 600U
#define HISTORY_COLUMNS 21U
#define HISTORY_MAX_AGE_MS 86400000U
typedef struct
{
    uint32_t timestamp_ms;
    uint32_t valid_mask;
    float values[HISTORY_COLUMNS];
    uint8_t accuracy, stabilized, run_in, reserved;
} HistoryPoint;
_Static_assert(sizeof(HistoryPoint) == 96U, "Browser history wire layout");

void History_Init(void);
uint32_t History_IntervalSeconds(uint32_t requested);
uint32_t History_Count(uint32_t now);
void History_Append(const MeasurementSnapshot* snapshot);
void History_Service(const MeasurementSnapshot* snapshot, uint32_t requested_interval_s);
/* Little-endian 32-byte header followed by up to 32 complete HistoryPoint records. */
size_t History_Export(uint8_t* destination, size_t capacity, uint32_t after, uint32_t now, uint32_t requested_interval_s);
void History_Values(const MeasurementSnapshot* snapshot, float values[HISTORY_COLUMNS]);
#endif
