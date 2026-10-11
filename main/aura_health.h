#pragma once
#include <stdint.h>

// Liveness counters: each long-running task counts its own loop passes. The
// desktop gets them with the reset reason (frame 0x33), so a task that stops
// shows by name while the others keep counting.
typedef enum {
    AURA_TASK_GAIT,
    AURA_TASK_CONTROL,
    AURA_TASK_FEEDBACK,
    AURA_TASK_PAD,
    AURA_TASK_COMMANDS,
    AURA_TASK_NETWORK,
    AURA_TASK_TELEMETRY,
    AURA_TASK_ESTIMATOR,
    AURA_TASK_SENSORS,
    AURA_TASK_COUNT
} aura_task_t;

void aura_health_beat(aura_task_t task);
uint32_t aura_health_beats(aura_task_t task);
// Least stack the task has had free since boot, in bytes (0 = not seen yet).
uint16_t aura_health_stack_free(aura_task_t task);
// Longest gait tick since the last take, in microseconds (50 Hz budget: 20 ms).
void aura_health_gait_tick_us(uint32_t us);
uint32_t aura_health_take_gait_tick_max_us(void);
