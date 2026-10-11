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
