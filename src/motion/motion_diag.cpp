#include "motion_diag.h"
#include "config_store.h"
#include "protocol_internal.h"

#include <string.h>

#ifndef HOST_TEST
#include <Arduino.h>
#endif

#define DIAG_MAGIC 0x4D444742u /* 'MDGB' — per-axis underrun / fifo_min */

static MotionDiag g_diag;

#if !defined(HOST_TEST)
/* Survive soft reboot / WDT (not power-cycle). */
static MotionDiag g_diag_noinit __attribute__((section(".noinit")));
static uint32_t g_diag_magic __attribute__((section(".noinit")));
#endif
static uint8_t g_restored_from_noinit;
#if !defined(HOST_TEST)
static uint32_t g_underrun_at_ms;
static uint8_t g_underrun_latched;
#endif

static void fifo_min_clear(void) {
  g_diag.fifo_min_level = 0xFFFFFFFFu;
  for (int i = 0; i < MOTION_DIAG_AXES; ++i) {
    g_diag.fifo_min_axis[i] = 0xFFFFFFFFu;
  }
}

static void diag_persist(void) {
#if !defined(HOST_TEST)
  g_diag_noinit = g_diag;
  g_diag_magic = DIAG_MAGIC;
#endif
}

void motion_diag_reset(void) {
  memset(&g_diag, 0, sizeof(g_diag));
  fifo_min_clear();
  g_restored_from_noinit = 0;
#if !defined(HOST_TEST)
  g_underrun_latched = 0;
  g_underrun_at_ms = 0;
#endif
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

void motion_diag_note_underrun(int axis) {
  ++g_diag.underrun_count;
  if (axis >= 0 && axis < MOTION_DIAG_AXES) {
    ++g_diag.underrun_axis[axis];
  }
#if !defined(HOST_TEST)
  g_underrun_latched = 1;
  g_underrun_at_ms = millis();
#endif
  diag_persist();
}

bool motion_diag_underrun_latched(void) {
#if !defined(HOST_TEST)
  return g_underrun_latched != 0 && (millis() - g_underrun_at_ms) < 1500u;
#else
  return false;
#endif
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

void motion_diag_note_fifo_level(int axis, unsigned level) {
  if (level < g_diag.fifo_min_level) {
    g_diag.fifo_min_level = level;
  }
  if (axis >= 0 && axis < MOTION_DIAG_AXES && level < g_diag.fifo_min_axis[axis]) {
    g_diag.fifo_min_axis[axis] = level;
  }
  diag_persist();
}

void motion_diag_boot_report(void) {
  if (!g_restored_from_noinit || config_get()->init_debug_level < 2) {
    return;
  }
  unsigned f0 = g_diag.fifo_min_axis[0];
  unsigned f1 = g_diag.fifo_min_axis[1];
  unsigned f2 = g_diag.fifo_min_axis[2];
  if (f0 == 0xFFFFFFFFu) {
    f0 = 0;
  }
  if (f1 == 0xFFFFFFFFu) {
    f1 = 0;
  }
  if (f2 == 0xFFFFFFFFu) {
    f2 = 0;
  }
  protocol_debug(2, "D:diag_restored underrun=%lu,%lu,%lu peak_hz=%.0f overshoot=%ld fifo_min=%u,%u,%u\n",
                 (unsigned long)g_diag.underrun_axis[0], (unsigned long)g_diag.underrun_axis[1],
                 (unsigned long)g_diag.underrun_axis[2], (double)g_diag.peak_step_hz,
                 (long)g_diag.overshoot_steps, f0, f1, f2);
#ifndef HOST_TEST
  if (rp2040.getResetReason() == RP2040::WDT_RESET) {
    protocol_debug(2, "D:reset=wdt\n");
  }
#endif
}
