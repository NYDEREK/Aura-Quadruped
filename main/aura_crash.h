#pragma once
#include <stdbool.h>
#include <stdint.h>

// The last crash the board stored (ELF core dump in the `coredump` flash
// partition), read once at boot so the desktop can name a reset's cause
// without a USB cable. Empty when core dumps are not enabled.
typedef struct {
    bool supported;          // core dumps to flash are built in
    bool present;            // a crash record is stored
    bool this_boot;          // this boot followed a crash or watchdog reset
    bool corrupted;          // backtrace could not be walked to the end
    uint8_t reset_reason;    // esp_reset_reason() of this boot
    char task[16];           // task running when it crashed
    uint32_t pc, cause, vaddr;
    uint8_t depth;
    uint32_t backtrace[16];
    char elf[9];             // first 8 hex digits of the crashed build's ELF SHA-256
    char reason[64];         // panic reason text
} aura_crash_t;

void aura_crash_init(void);
const aura_crash_t *aura_crash_get(void);
