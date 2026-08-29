#ifndef TASK_HEALTH_H
#define TASK_HEALTH_H

#include <stdint.h>

/**
 * Liveness tracking for the tasks the watchdog must protect.
 *
 * The default task previously refreshed the IWDG unconditionally, so it kept
 * the watchdog happy even when every other task was wedged. Each essential
 * task now reports in once per loop iteration and the default task refreshes
 * the IWDG only while all of them are within their deadline; if one stops
 * reporting, the refresh stops and the watchdog resets the device.
 *
 * Deliberately free of RTOS dependencies: the caller supplies the tick, which
 * keeps the module testable on the host. Every field is a naturally aligned
 * 32-bit word written by exactly one task, so no locking is required.
 */

typedef enum
{
    TASK_HEALTH_USER_INTERFACE = 0,
    TASK_HEALTH_SENSORS,
    TASK_HEALTH_BME680,
    TASK_HEALTH_WIFI,
    TASK_HEALTH_COUNT
} TaskHealthId;

/** Forgets every registration; intended for tests and for a clean startup. */
void TaskHealth_Reset(void);

/**
 * Marks a task as essential and starts its deadline running from now_ms, so a
 * task that never reaches its first loop iteration is detected too.
 */
void TaskHealth_Register(TaskHealthId id, uint32_t deadline_ms, uint32_t now_ms);

void TaskHealth_ReportAlive(TaskHealthId id, uint32_t now_ms);

/** Bitmask of registered tasks that have missed their deadline. */
uint32_t TaskHealth_LateMask(uint32_t now_ms);

/** Returns 1 when every registered task reported within its deadline. */
uint8_t TaskHealth_AllAlive(uint32_t now_ms);

#endif /* TASK_HEALTH_H */
