#pragma once
// Independent build switches. Rebuild and flash after changing these values.
#define AURA_ENABLE_WIFI 1
#define AURA_ENABLE_DUALSENSE 1

// Bluetooth (controller + Bluedroid) is pinned to core 0 at the highest
// priorities and takes ~40 % of it with the pad's ~200 reports/s. Unpinned
// motion tasks preempted there waited up to 140 ms while core 1 idled, and
// core 0 at 100 % for 5 s is a task-watchdog reset. Gait, servo control,
// servo feedback and the estimator run on core 1 (Wi-Fi's core, ~20 %).
#define AURA_MOTION_CORE 1
