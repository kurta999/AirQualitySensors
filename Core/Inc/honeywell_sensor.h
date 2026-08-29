#ifndef HONEYWELL_SENSOR_H
#define HONEYWELL_SENSOR_H

#include "stm32l4xx_hal.h"

/** Restart reception when no valid frame has arrived for this long. */
#define HONEYWELL_SILENCE_TIMEOUT_MS 10000U

/**
 * Sends the start commands and arms interrupt-driven reception. On any
 * failure the sensor is left marked as not receiving, so Honeywell_Poll()
 * retries; a missing particle sensor never stops the rest of the system.
 */
HAL_StatusTypeDef Honeywell_Init(void);

/** UART4 receive-complete handler; call from HAL_UART_RxCpltCallback. */
void Honeywell_UartRecv(UART_HandleTypeDef* huart);

/**
 * UART4 error handler; call from HAL_UART_ErrorCallback. The HAL aborts
 * reception on overrun/noise/framing errors, so without this the driver would
 * never rearm and particle readings would stay frozen forever.
 */
void Honeywell_UartError(UART_HandleTypeDef* huart);

/** Forces Honeywell_Poll() to reinitialise the sensor on its next call. */
void Honeywell_RequestRestart(void);

/**
 * Rearms reception if it stopped, and fully reinitialises the sensor if no
 * frame has been decoded for HONEYWELL_SILENCE_TIMEOUT_MS. Call periodically
 * from a task.
 */
void Honeywell_Poll(uint32_t now_ms);

#endif /* HONEYWELL_SENSOR_H */
