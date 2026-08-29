#ifndef SENSOR_RETRY_H
#define SENSOR_RETRY_H

#include <stdint.h>

/**
 * Per-sensor init/read state machine. Each sensor owns one of these so that a
 * sensor that is absent, unpowered or failing is retried on its own without
 * blocking any other sensor on the same task.
 */

/** Consecutive failed reads before a sensor is dropped and re-initialised. */
#define SENSOR_MAX_READ_FAILURES 5U

typedef struct
{
    uint8_t initialized;
    uint8_t failure_count;
} SensorRetryState;

static inline void SensorRetry_Reset(SensorRetryState* state)
{
    state->initialized = 0U;
    state->failure_count = 0U;
}

static inline void SensorRetry_ReportInitialized(SensorRetryState* state)
{
    state->initialized = 1U;
    state->failure_count = 0U;
}

static inline uint8_t SensorRetry_IsInitialized(const SensorRetryState* state)
{
    return state->initialized;
}

static inline void SensorRetry_ReportSuccess(SensorRetryState* state)
{
    state->failure_count = 0U;
}

/**
 * Records one failed read. Returns 1 when the failure budget is exhausted and
 * the caller should invalidate the reading and re-initialise the sensor.
 */
static inline uint8_t SensorRetry_ReportFailure(SensorRetryState* state)
{
    if(state->failure_count < SENSOR_MAX_READ_FAILURES)
    {
        ++state->failure_count;
    }
    if(state->failure_count >= SENSOR_MAX_READ_FAILURES)
    {
        SensorRetry_Reset(state);
        return 1U;
    }
    return 0U;
}

#endif /* SENSOR_RETRY_H */
