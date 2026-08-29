#ifndef SCD_SENSOR_H
#define SCD_SENSOR_H

#include <stdint.h>

/** Returns 0 on success, non-zero on failure. */
int16_t Scd30_Init(void);

/**
 * Checks once whether a new measurement is available and stores it if so.
 * Unlike a blocking read this never holds the shared I2C bus (or the calling
 * task) for a whole measurement interval, so the caller can simply poll it
 * from its normal loop.
 *
 * Returns 0 when the sensor answered (whether or not data was ready) and a
 * negative value on a bus or sensor error.
 */
int16_t Scd30_Poll(void);

#endif /* SCD_SENSOR_H */
