#include "aura_crash.h"

#include <stdlib.h>
#include <string.h>

#include "esp_system.h"
#include "sdkconfig.h"

#if CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH
#include "esp_core_dump.h"
#endif

static aura_crash_t crash;

void aura_crash_init(void)
{
    memset(&crash, 0, sizeof(crash));
    const esp_reset_reason_t reason = esp_reset_reason();
    crash.reset_reason = (uint8_t)reason;
    crash.this_boot = reason == ESP_RST_PANIC || reason == ESP_RST_INT_WDT ||
                      reason == ESP_RST_TASK_WDT || reason == ESP_RST_WDT;
#if CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH
    crash.supported = true;
    size_t address = 0, size = 0;
    if (esp_core_dump_image_get(&address, &size) != ESP_OK || size == 0) return;
    // The summary is ~300 bytes; keep it off the caller's stack.
    esp_core_dump_summary_t *summary = malloc(sizeof(*summary));
    if (!summary) return;
    if (esp_core_dump_get_summary(summary) == ESP_OK) {
        crash.present = true;
        strncpy(crash.task, summary->exc_task, sizeof(crash.task) - 1);
        crash.pc = summary->exc_pc;
        crash.cause = summary->ex_info.exc_cause;
        crash.vaddr = summary->ex_info.exc_vaddr;
        const uint32_t depth = summary->exc_bt_info.depth;
        crash.depth = (uint8_t)(depth > 16 ? 16 : depth);
        memcpy(crash.backtrace, summary->exc_bt_info.bt, crash.depth * sizeof(uint32_t));
        crash.corrupted = summary->exc_bt_info.corrupted;
        memcpy(crash.elf, summary->app_elf_sha256, sizeof(crash.elf) - 1);
        if (esp_core_dump_get_panic_reason(crash.reason, sizeof(crash.reason)) != ESP_OK)
            crash.reason[0] = '\0';
    }
    free(summary);
#endif
}

const aura_crash_t *aura_crash_get(void)
{
    return &crash;
}
