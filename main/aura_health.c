#include "aura_health.h"

#include <string.h>

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

static volatile uint32_t section_max_us[AURA_GAIT_SECTION_COUNT];

void aura_health_gait_section_us(aura_gait_section_t section, uint32_t us)
{
    if (section < AURA_GAIT_SECTION_COUNT && us > section_max_us[section]) section_max_us[section] = us;
}

void aura_health_take_gait_sections_us(uint32_t out[AURA_GAIT_SECTION_COUNT])
{
    for (int i = 0; i < AURA_GAIT_SECTION_COUNT; ++i) {
        out[i] = section_max_us[i];
        section_max_us[i] = 0;
    }
}

// Run-time counters are esp_timer microseconds (uint32_t, wraps after ~71
// minutes; unsigned differences stay right across one wrap).
#define CPU_TASK_SLOTS 36
static TaskStatus_t cpu_tasks[CPU_TASK_SLOTS];
static struct {
    TaskHandle_t handle;
    uint32_t counter;
} cpu_previous[CPU_TASK_SLOTS];
static unsigned cpu_previous_count;
static uint32_t cpu_previous_total, cpu_previous_gait_beats;

static bool previous_counter(TaskHandle_t handle, uint32_t *counter)
{
    for (unsigned i = 0; i < cpu_previous_count; ++i)
        if (cpu_previous[i].handle == handle) {
            *counter = cpu_previous[i].counter;
            return true;
        }
    return false;
}

void aura_health_sample_cpu(aura_cpu_stats_t *out)
{
    memset(out, 0, sizeof(*out));
    uint32_t total = 0;
    const UBaseType_t count = uxTaskGetSystemState(cpu_tasks, CPU_TASK_SLOTS, &total);
    if (count == 0) return;   // more tasks than slots
    const uint32_t window = total - cpu_previous_total;
    const uint32_t gait_beats = beats[AURA_TASK_GAIT];
    const bool have_window = cpu_previous_count > 0 && window > 0;
    TaskHandle_t idle[2] = {xTaskGetIdleTaskHandleForCore(0), xTaskGetIdleTaskHandleForCore(1)};
    uint32_t idle_time[2] = {0, 0};
    for (UBaseType_t i = 0; have_window && i < count; ++i) {
        uint32_t before;
        if (!previous_counter(cpu_tasks[i].xHandle, &before)) continue;
        const uint32_t used = cpu_tasks[i].ulRunTimeCounter - before;
        const uint64_t percent64 = ((uint64_t)used * 100u + window / 2) / window;
        const uint8_t percent = (uint8_t)(percent64 > 100u ? 100u : percent64);
        if (cpu_tasks[i].xHandle == idle[0] || cpu_tasks[i].xHandle == idle[1]) {
            idle_time[cpu_tasks[i].xHandle == idle[1]] = used;
            continue;
        }
        const char *name = cpu_tasks[i].pcTaskName ? cpu_tasks[i].pcTaskName : "";
        if (strcmp(name, "robot_gait") == 0 && gait_beats != cpu_previous_gait_beats) {
            const uint32_t per_tick = used / (gait_beats - cpu_previous_gait_beats);
            out->gait_tick_cpu_us = (uint16_t)(per_tick > 0xffffu ? 0xffffu : per_tick);
        }
        // Keep the busiest few, largest first.
        int slot = out->top_count;
        while (slot > 0 && out->top[slot - 1].percent < percent) --slot;
        if (slot >= AURA_CPU_TOP_TASKS) continue;
        const int last = out->top_count < AURA_CPU_TOP_TASKS ? out->top_count : AURA_CPU_TOP_TASKS - 1;
        for (int j = last; j > slot; --j) out->top[j] = out->top[j - 1];
        aura_cpu_task_t *entry = &out->top[slot];
        memset(entry, 0, sizeof(*entry));
        strncpy(entry->name, name, sizeof(entry->name) - 1);
        entry->percent = percent;
        const BaseType_t core = xTaskGetCoreID(cpu_tasks[i].xHandle);
        entry->core = core == 0 || core == 1 ? (int8_t)core : -1;
        if (out->top_count < AURA_CPU_TOP_TASKS) ++out->top_count;
    }
    if (have_window) {
        for (int core = 0; core < 2; ++core) {
            const uint32_t idle_percent = (uint32_t)(((uint64_t)idle_time[core] * 100u + window / 2) / window);
            out->core_load_percent[core] = (uint8_t)(idle_percent >= 100u ? 0u : 100u - idle_percent);
        }
        const uint64_t ticks = (uint64_t)(gait_beats - cpu_previous_gait_beats) * 1000000u / window;
        out->gait_ticks_per_s = (uint16_t)(ticks > 0xffffu ? 0xffffu : ticks);
        out->valid = true;
    }
    cpu_previous_count = count;
    for (UBaseType_t i = 0; i < count; ++i) {
        cpu_previous[i].handle = cpu_tasks[i].xHandle;
        cpu_previous[i].counter = cpu_tasks[i].ulRunTimeCounter;
    }
    cpu_previous_total = total;
    cpu_previous_gait_beats = gait_beats;
}
