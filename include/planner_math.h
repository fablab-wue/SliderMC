#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define PLANNER_PACK_MAX 64
/*
 * Below this rate every word carries a single pulse (finest ramp resolution).
 * It must stay low enough that the 8-word TX FIFO still buffers the time
 * budget: 8 / 2500 Hz = 3.2 ms. Raising it starves the FIFO at the crossover.
 */
#define PLANNER_PACK_MIN_HZ 2500
#define PLANNER_FIFO_TIME_BUDGET_MS 3.0f

/** Max |v| (mm/s) that can stop within dist_mm: d = π v² / (4 a). */
float planner_vmax_for_distance(float dist_mm, float accel_mm_s2);

/**
 * Pack size 0..64. remaining_steps <= 0 → 0 (pendeln guard) before any shortcut.
 * pending_steps = steps already in TX shadow for budget accounting.
 */
int planner_pack_n(float step_hz, int remaining_steps, int pending_steps);

/** Delay cycles so period ≈ fixed_overhead + delay. Pass PIO_STEP_PERIOD_FIXED. */
uint32_t planner_hz_to_delay(float step_hz, uint32_t sysclk_hz, uint32_t fixed_cycles);

/** Max achievable step rate for fixed_cycles + min delay 1. */
float planner_max_step_hz(uint32_t sysclk_hz, uint32_t fixed_cycles);

/** Raised-cosine blend of velocity from v0 toward v1 over phase phi in [0,1]. */
float planner_sine_vel(float v0, float v1, float phi);

/**
 * Advance phase for a sine ramp given peak accel a and |v1-v0|.
 * Returns new phi clamped to [0,1]. dt_s > 0.
 *
 * Use `double` for the phase accumulator to reduce single-precision
 * accumulation error over many small dt steps.
 */
double planner_sine_advance_phi(double phi, float v0, float v1, float accel_mm_s2, double dt_s);

/** Steps to bleed |vel| via sine stop-distance law (ceil); 0 if already stopped. */
int planner_stop_rem_steps(float vel_mm_s, float accel_mm_s2, float steps_per_mm);

/**
 * True when a new target sign opposes travel and speed is still significant —
 * fill path must decelerate in place before reversing.
 */
bool planner_needs_reverse_decel(int last_sign, int target_sign, float vel_mm_s);

#ifdef __cplusplus
}
#endif
