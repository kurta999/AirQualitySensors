#ifndef I2C_BUS_H
#define I2C_BUS_H

#include "stm32l4xx_hal.h"

/**
 * Serialises access to I2C1, which is shared by the VEML6070 and SCD30
 * (SensorsTask) and the TCS34725 (SensorsBme680Task). Those tasks run at
 * different priorities, so without this lock the higher-priority task can
 * preempt a transfer in progress and the HAL rejects the second one with
 * HAL_BUSY.
 *
 * The lock is NOT recursive: acquire it once around a complete sensor
 * operation, never around the individual HAL calls inside one.
 */

/** Bound on how long a task waits for the bus before giving up. */
#define I2C_BUS_LOCK_TIMEOUT_MS 1000U

/** Creates the lock. Must be called before any task that uses I2C1 starts. */
void I2cBus_Init(void);

/** Returns HAL_OK when the bus was acquired, HAL_BUSY on timeout. */
HAL_StatusTypeDef I2cBus_Lock(void);

void I2cBus_Unlock(void);

#endif /* I2C_BUS_H */
