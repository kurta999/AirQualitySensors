#ifndef DEVICE_CONFIG_H
#define DEVICE_CONFIG_H
#include <stddef.h>
#include <stdint.h>

/* User settings are separate from build-time peripheral capabilities. */
typedef struct
{
    uint32_t version;
    uint32_t master_enabled;
    uint32_t sensor_mask;
    uint32_t sensor_poll_ms;
    uint32_t light_poll_ms;
    uint32_t send_interval_ms;
    uint32_t receiver_enabled;
    uint32_t receiver_port;
    uint32_t history_interval_s; /* 0 = automatically retain 24 hours */
    uint32_t scd_interval_s;
    uint32_t scd_pressure_mbar;
    uint32_t scd_temperature_offset; /* hundredths of a degree C */
    uint32_t scd_altitude_m;
    uint32_t scd_asc;
    uint32_t scd_recovery;
    int32_t bsec_heat_offset; /* hundredths of a degree C */
    uint32_t bsec_save_interval_s;
    uint32_t light_gain;
    uint32_t light_cycles; /* 2.4 ms per cycle */
    uint32_t uv_integration;
    char receiver_ip[16];
    char hostname[32];
    char device_name[32];
} DeviceConfig;

void DeviceConfig_Defaults(DeviceConfig* config);
int DeviceConfig_Validate(const DeviceConfig* config);
/* Parse a bounded form patch atomically: failure leaves output unchanged. */
int DeviceConfig_Parse(const DeviceConfig* current, const char* form, size_t length,
                       DeviceConfig* output);
void DeviceConfig_Init(void);
void DeviceConfig_Get(DeviceConfig* config);
uint32_t DeviceConfig_Revision(void);
uint32_t DeviceConfig_Sequence(void);
int DeviceConfig_Save(const DeviceConfig* config);
#endif
