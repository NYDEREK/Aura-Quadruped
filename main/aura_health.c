#include "aura_health.h"

// One writer per counter (its own task); readers only copy, so no lock.
static volatile uint32_t beats[AURA_TASK_COUNT];

void aura_health_beat(aura_task_t task)
{
    if (task < AURA_TASK_COUNT) beats[task]++;
}

uint32_t aura_health_beats(aura_task_t task)
{
    return task < AURA_TASK_COUNT ? beats[task] : 0;
}
