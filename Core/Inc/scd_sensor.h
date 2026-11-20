#ifndef SCD_SENSOR_H
#define SCD_SENSOR_H

#include <stdint.h>

/* Debugger-visible diagnostics. Read while halted for a consistent view. */
typedef struct
{
    uint16_t firmware_version;
    uint16_t measurement_interval_s;
    uint16_t data_ready;
    int16_t init_status;
    int16_t poll_status;
    uint32_t reset_count;
    uint32_t sample_count;
    uint32_t not_ready_count;
    uint32_t last_sample_ms;
} Scd30Diagnostics;

extern volatile Scd30Diagnostics scd30_diagnostics;

/** Returns 0 on success, non-zero on failure. */
int16_t Scd30_Init(void);

/**
 * Checks once whether a new measurement is available and stores it if so.
 * Unlike a blocking read this never holds the shared I2C bus (or the calling
 * task) for a whole measurement interval, so the caller can simply poll it
 * from its normal loop.
 *
 * Returns 0 when the sensor answered (whether or not data was ready) and a
 * non-zero value on a bus or sensor error. With recovery enabled, -2 means
 * no measurement arrived for 30 seconds, requesting reinitialization.
 */
int16_t Scd30_Poll(void);

#endif /* SCD_SENSOR_H */
