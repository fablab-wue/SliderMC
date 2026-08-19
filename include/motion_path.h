#pragma once

#include "motion_api.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Motion Path — a 2nd, simpler planner for host-authored motion paths.
 *
 * The host streams signed delta-distance samples (um, int16) via PD into a
 * flat RAM buffer at a fixed time-slice (PS, us). PG plays the buffer back:
 * each slice's distance is converted to a step count (steps_per_mm) and the
 * slice time to a PIO delay, using error-diffusion accumulators so rounding
 * never biases total distance or total time. A 0 sample means "stand still"
 * for that slice (no STEP word is issued; the PIO naturally holds output).
 *
 * On MS/H, or when the buffer is exhausted, path-mode hands its last
 * position/velocity to the main planner (planner_takeover_from_path) and
 * ends, so the normal soft-stop/halt decel takes over from that speed.
 */

void motion_path_init(void);

bool motion_path_clear(void);              /* PC; false if active */
bool motion_path_add(int16_t distance_um); /* PD; false if active or full */
uint32_t motion_path_count(void);          /* PN */

bool motion_path_set_slice_us(uint32_t us); /* PS <value>; false if active or < min */
uint32_t motion_path_get_slice_us(void);

bool motion_path_go(void); /* PG; false if active, empty, or disabled */
bool motion_path_is_active(void);

/** Called by the motion feed task instead of planner_fill_fifo() while active. */
int motion_path_fill_fifo(void);

/** Seed the main planner from the current path position/velocity and end path-mode. */
void motion_path_abort_to_planner(void);

void motion_path_get_status(McStatus *out);

/**
 * Pure math (host-testable), no hardware access:
 * fractional-step carry so sum(steps) tracks sum(distance) * steps_per_mm.
 * distance_um == 0 always yields 0 steps and leaves *step_err untouched.
 */
int32_t motion_path_diffuse_steps(int16_t distance_um, float steps_per_mm, double *step_err);

/**
 * Pure math (host-testable): fractional-cycle carry so sum(cycles) tracks
 * sum(slice_us) * sysclk_hz.
 */
uint32_t motion_path_diffuse_cycles(uint32_t slice_us, uint32_t sysclk_hz, double *time_err);

#ifdef __cplusplus
}
#endif
