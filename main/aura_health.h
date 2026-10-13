#pragma once
#include <stdbool.h>
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

// Parts of one gait tick, timed separately to find what makes it slow.
typedef enum {
    AURA_GAIT_SECTION_INPUT,      // pad input and servo feedback snapshot
    AURA_GAIT_SECTION_ATTITUDE,   // IMU posture loop
    AURA_GAIT_SECTION_PLAN,       // gait phase, foot and torso planning
    AURA_GAIT_SECTION_JOINTS,     // IK and the twelve servo targets
    AURA_GAIT_SECTION_ESTIMATE,   // estimator legs and odometry
    AURA_GAIT_SECTION_COUNT
} aura_gait_section_t;
void aura_health_gait_section_us(aura_gait_section_t section, uint32_t us);
// Longest time of each section since the last take, microseconds.
void aura_health_take_gait_sections_us(uint32_t out[AURA_GAIT_SECTION_COUNT]);

// CPU use since the previous sample, from FreeRTOS run-time counters. The
// telemetry task samples it once a second.
#define AURA_CPU_TOP_TASKS 4
typedef struct {
    char name[16];
    uint8_t percent;        // of one core
    int8_t core;            // pinned core, -1 = either
} aura_cpu_task_t;

typedef struct {
    bool valid;
    uint8_t core_load_percent[2];
    uint16_t gait_tick_cpu_us;   // CPU time the gait task really used per tick
    uint16_t gait_ticks_per_s;
    uint8_t top_count;
    aura_cpu_task_t top[AURA_CPU_TOP_TASKS];   // busiest tasks, idle excluded
} aura_cpu_stats_t;

void aura_health_sample_cpu(aura_cpu_stats_t *out);
// The most recent sample, for readers other than the sampling task.
void aura_health_cpu_latest(aura_cpu_stats_t *out);
