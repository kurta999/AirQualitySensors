#include "task_health.h"

typedef struct
{
    uint32_t deadline_ms;
    uint32_t last_alive_ms;
    uint32_t registered;
} TaskHealthEntry;

static TaskHealthEntry entries[TASK_HEALTH_COUNT];

void TaskHealth_Reset(void)
{
    for(uint32_t index = 0U; index < (uint32_t)TASK_HEALTH_COUNT; ++index)
    {
        entries[index].deadline_ms = 0U;
        entries[index].last_alive_ms = 0U;
        entries[index].registered = 0U;
    }
}

void TaskHealth_Register(TaskHealthId id, uint32_t deadline_ms, uint32_t now_ms)
{
    if(id >= TASK_HEALTH_COUNT || deadline_ms == 0U)
    {
        return;
    }

    entries[id].deadline_ms = deadline_ms;
    entries[id].last_alive_ms = now_ms;
    entries[id].registered = 1U;
}

void TaskHealth_ReportAlive(TaskHealthId id, uint32_t now_ms)
{
    if(id >= TASK_HEALTH_COUNT)
    {
        return;
    }
    entries[id].last_alive_ms = now_ms;
}

uint32_t TaskHealth_LateMask(uint32_t now_ms)
{
    uint32_t late = 0U;

    for(uint32_t index = 0U; index < (uint32_t)TASK_HEALTH_COUNT; ++index)
    {
        if(entries[index].registered == 0U)
        {
            continue;
        }

        /* Unsigned subtraction stays correct across the tick wraparound. */
        const uint32_t elapsed = now_ms - entries[index].last_alive_ms;
        if(elapsed > entries[index].deadline_ms)
        {
            late |= (uint32_t)1U << index;
        }
    }
    return late;
}

uint8_t TaskHealth_AllAlive(uint32_t now_ms)
{
    return TaskHealth_LateMask(now_ms) == 0U ? 1U : 0U;
}
