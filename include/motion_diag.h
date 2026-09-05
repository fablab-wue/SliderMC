#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MOTION_DIAG_AXES 3

typedef struct {
  uint32_t underrun_count;
  uint32_t underrun_axis[MOTION_DIAG_AXES];
  float peak_step_hz;
  int32_t overshoot_steps;
  unsigned fifo_min_level;
  unsigned fifo_min_axis[MOTION_DIAG_AXES];
} MotionDiag;

void motion_diag_reset(void);
/** Restore counters from .noinit if prior run left a valid snapshot (e.g. after WDT). */
void motion_diag_init_from_noinit(void);
bool motion_diag_restored_after_reset(void);
void motion_diag_get(MotionDiag *out);

void motion_diag_note_underrun(int axis);
void motion_diag_note_hz(float step_hz);
void motion_diag_note_overshoot(int steps);
void motion_diag_note_fifo_level(int axis, unsigned level);

/** USB debug line if diag was restored and init_debug_level>=2 (call after protocol_init). */
void motion_diag_boot_report(void);

#ifdef __cplusplus
}
#endif
