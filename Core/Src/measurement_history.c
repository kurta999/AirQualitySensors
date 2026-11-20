#include "app_config.h"
#if APP_USB_RNDIS_ENABLED || defined(AQS_HISTORY_HOST_TEST)
#include "measurement_history.h"
#include <string.h>

/* This NOLOAD region has no startup clearing cost. count=0 prevents reading
 * uninitialized entries; every byte of a point is written before publication. */
#ifndef AQS_HISTORY_HOST_TEST
__attribute__((section(".history_ram2"), aligned(4)))
#endif
static HistoryPoint points[HISTORY_CAPACITY];
static uint32_t head, count, sequence, last_sample, interval;

void History_Init(void) { head = count = sequence = last_sample = interval = 0; }
uint32_t History_IntervalSeconds(uint32_t requested)
{
    if(requested != 0U) { return requested; }
    /* Round up to 30 s, allowing a whole day within the fixed RAM budget. */
    return ((86400U + (HISTORY_CAPACITY - 2U)) / (HISTORY_CAPACITY - 1U) + 29U) / 30U * 30U;
}
static void prune(uint32_t now)
{
    while(count != 0U && (uint32_t)(now - points[head].timestamp_ms) > HISTORY_MAX_AGE_MS)
    { head = (head + 1U) % HISTORY_CAPACITY; --count; }
}
uint32_t History_Count(uint32_t now) { prune(now); return count; }

void History_Values(const MeasurementSnapshot* m, float v[HISTORY_COLUMNS])
{
    v[0] = m->scd30_co2_ppm; v[1] = m->scd30_temperature_c; v[2] = m->scd30_relative_humidity_percent;
    v[3] = m->bme680_temperature_c; v[4] = m->bme680_relative_humidity_percent;
    v[5] = m->bme680_pressure_hpa; v[6] = (float)m->bme680_gas_resistance_ohm;
    v[7] = (float)m->uv_raw;
    v[8] = m->red; v[9] = m->green; v[10] = m->blue;
    v[11] = (float)m->color_temperature_k; v[12] = (float)m->lux;
    v[13] = m->air_quality.index; v[14] = m->air_quality.static_index;
    v[15] = m->air_quality.co2_equivalent_ppm; v[16] = m->air_quality.breath_voc_equivalent_ppm;
    v[17] = m->air_quality.temperature_c; v[18] = m->air_quality.relative_humidity_percent;
    v[19] = (float)m->pm25_ug_m3; v[20] = (float)m->pm10_ug_m3;
}
void History_Append(const MeasurementSnapshot* m)
{
    prune(m->snapshot_timestamp_ms);
    if(count == HISTORY_CAPACITY) { head = (head + 1U) % HISTORY_CAPACITY; --count; }
    HistoryPoint* p = &points[(head + count) % HISTORY_CAPACITY];
    p->timestamp_ms = m->snapshot_timestamp_ms;
    p->valid_mask = m->valid_mask;
    History_Values(m, p->values);
    p->accuracy = m->air_quality.accuracy;
    p->stabilized = m->air_quality.stabilized;
    p->run_in = m->air_quality.run_in;
    p->reserved = 0;
    ++count; ++sequence;
    last_sample = m->snapshot_timestamp_ms;
}
void History_Service(const MeasurementSnapshot* m, uint32_t requested)
{
    const uint32_t current_interval = History_IntervalSeconds(requested);
    if(current_interval != interval)
    { head = count = last_sample = 0; interval = current_interval; }
    prune(m->snapshot_timestamp_ms);
    if(m->valid_mask != 0U && (count == 0U || (uint32_t)(m->snapshot_timestamp_ms - last_sample) >= interval * 1000U))
    { History_Append(m); }
}
size_t History_Export(uint8_t* destination, size_t capacity, uint32_t after, uint32_t now, uint32_t requested)
{
    if(capacity < 32U) { return 0; }
    prune(now);
    uint32_t start = 0;
    const uint32_t first = sequence - count + 1U;
    if(after != 0U)
    {
        while(start < count && (int32_t)((first + start) - after) <= 0) { ++start; }
    }
    uint32_t n = count - start;
    const uint32_t maximum = (uint32_t)((capacity - 32U) / sizeof(HistoryPoint));
    if(n > 32U) { n = 32U; }
    if(n > maximum) { n = maximum; }
    const uint32_t header[8] = {UINT32_C(0x31524941), 1U, sizeof(HistoryPoint), HISTORY_COLUMNS,
        n, now, History_IntervalSeconds(requested), n ? first + start : 0U};
    memcpy(destination, header, sizeof(header));
    for(uint32_t i = 0; i < n; ++i)
    { memcpy(destination + 32U + i * sizeof(HistoryPoint), &points[(head + start + i) % HISTORY_CAPACITY], sizeof(HistoryPoint)); }
    return 32U + n * sizeof(HistoryPoint);
}
#endif
