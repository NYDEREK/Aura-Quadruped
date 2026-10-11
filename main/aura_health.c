#include "aura_health.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// One writer per counter (its own task); readers only copy, so no lock.
static volatile uint32_t beats[AURA_TASK_COUNT];
static volatile uint16_t stack_free[AURA_TASK_COUNT];
static volatile uint32_t gait_tick_max_us;

void aura_health_beat(aura_task_t task)
{
    if (task >= AURA_TASK_COUNT) return;
    // Every 32 passes the task notes how little of its stack has ever been
    // free (FreeRTOS high-water mark, bytes on ESP-IDF).
    if ((beats[task] & 31u) == 0) {
        const UBaseType_t free_bytes = uxTaskGetStackHighWaterMark(NULL);
        stack_free[task] = free_bytes > 0xffffu ? 0xffffu : (uint16_t)free_bytes;
    }
    beats[task]++;
}

uint32_t aura_health_beats(aura_task_t task)
{
    return task < AURA_TASK_COUNT ? beats[task] : 0;
}

uint16_t aura_health_stack_free(aura_task_t task)
{
    return task < AURA_TASK_COUNT ? stack_free[task] : 0;
}

void aura_health_gait_tick_us(uint32_t us)
{
    if (us > gait_tick_max_us) gait_tick_max_us = us;
}

uint32_t aura_health_take_gait_tick_max_us(void)
{
    const uint32_t value = gait_tick_max_us;
    gait_tick_max_us = 0;
    return value;
}
