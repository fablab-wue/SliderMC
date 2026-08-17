#include "motion_diag.h"
#include "config_store.h"
#include "protocol_internal.h"

#include <string.h>

#ifndef HOST_TEST
#include <Arduino.h>
#endif

#define DIAG_MAGIC 0x4D444741u /* 'MDGA' */

static MotionDiag g_diag;

#if !defined(HOST_TEST)
/* Survive soft reboot / WDT (not power-cycle). */
static MotionDiag g_diag_noinit __attribute__((section(".noinit")));
static uint32_t g_diag_magic __attribute__((section(".noinit")));
#endif
static uint8_t g_restored_from_noinit;

static void diag_persist(void) {
#if !defined(HOST_TEST)
  g_diag_noinit = g_diag;
  g_diag_magic = DIAG_MAGIC;
#endif
}

void motion_diag_reset(void) {
  memset(&g_diag, 0, sizeof(g_diag));
  g_diag.fifo_min_level = 0xFFFFFFFFu;
  g_restored_from_noinit = 0;
  diag_persist();
}

void motion_diag_init_from_noinit(void) {
#if !defined(HOST_TEST)
  /* Only a watchdog reset needs a post-mortem; every other boot starts clean so
     the ID counters always describe the running session. */
  if (g_diag_magic == DIAG_MAGIC && rp2040.getResetReason() == RP2040::WDT_RESET) {
    g_diag = g_diag_noinit;
    g_restored_from_noinit = 1;
    return;
  }
#endif
  motion_diag_reset();
}

bool motion_diag_restored_after_reset(void) { return g_restored_from_noinit != 0; }

void motion_diag_get(MotionDiag *out) {
  if (out) {
    *out = g_diag;
  }
}

void motion_diag_note_underrun(void) {
  ++g_diag.underrun_count;
  diag_persist();
}

void motion_diag_note_hz(float step_hz) {
  if (step_hz > g_diag.peak_step_hz) {
    g_diag.peak_step_hz = step_hz;
    diag_persist();
  }
}

void motion_diag_note_overshoot(int steps) {
  if (steps > 0) {
    g_diag.overshoot_steps += steps;
    diag_persist();
  }
}

void motion_diag_note_fifo_level(unsigned level) {
  if (level < g_diag.fifo_min_level) {
    g_diag.fifo_min_level = level;
    diag_persist();
  }
}

void motion_diag_boot_report(void) {
  if (!g_restored_from_noinit || config_get()->init_debug_level < 2) {
    return;
  }
  unsigned fifo_min = g_diag.fifo_min_level;
  if (fifo_min == 0xFFFFFFFFu) {
    fifo_min = 0;
  }
  protocol_debug(2, "D:diag_restored underrun=%lu peak_hz=%.0f overshoot=%ld fifo_min=%u\n",
                 (unsigned long)g_diag.underrun_count, (double)g_diag.peak_step_hz,
                 (long)g_diag.overshoot_steps, fifo_min);
#ifndef HOST_TEST
  if (rp2040.getResetReason() == RP2040::WDT_RESET) {
    protocol_debug(2, "D:reset=wdt\n");
  }
#endif
}
