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
 * 8 words at this rate last 3.2 ms. Axes 3 and 4 pack from 1000 Hz so the
 * one-pulse storm stays inside the feed task.
 */
#define PLANNER_PACK_MIN_HZ 2500
/* 3-axis n=1 at 2500 Hz is ~7500 word/s of soft-float; pack earlier. */
#define PLANNER_PACK_MIN_HZ_3AXIS 1000
/* Queued step time the filler tries to keep ahead of the PIO. */
#define PLANNER_FIFO_HORIZON_MS 6.0f
/* Longest single word. Several words then cover the horizon, so a pull
 * does not empty the FIFO down to one burst. */
#define PLANNER_FIFO_WORD_MS 2.0f
#define PLANNER_FIFO_TIME_BUDGET_MS PLANNER_FIFO_WORD_MS

/** Max |v| (mm/s) that can stop within dist_mm: d = π v² / (4 a). */
float planner_vmax_for_distance(float dist_mm, float accel_mm_s2);

/** Triangle (no-cruise) peak |v|: D = π v²/4 · (1/a_accel + 1/a_decel). */
float planner_vmax_triangle(float dist_mm, float accel_mm_s2, float decel_mm_s2);

/**
 * Pack size 0..64. remaining_steps <= 0 → 0 (pendeln guard) before any shortcut.
 * pending_steps = steps already in TX shadow for budget accounting.
 */
int planner_pack_n(float step_hz, int remaining_steps, int pending_steps);
int planner_pack_n_min(float step_hz, int remaining_steps, int pending_steps,
                       int pack_min_hz);

/** Delay cycles so period ≈ fixed_overhead + delay. Pass PIO_STEP_PERIOD_FIXED. */
uint32_t planner_hz_to_delay(float step_hz, uint32_t sysclk_hz, uint32_t fixed_cycles);

/** Max achievable step rate for fixed_cycles + min delay 1. */
float planner_max_step_hz(uint32_t sysclk_hz, uint32_t fixed_cycles);

/**
 * Quarter-wave LUT + linear interpolate. Same law as libm sinf/cosf on the
 * planner's [0, π] arguments; ~5× cheaper on Cortex-M0+ soft-float.
 */
float planner_sinf(float x);
float planner_cosf(float x);

/** sin(unit * π/2) for unit in [0, 1]. Brake arc indexes this by R/D. */
float planner_sinf_unit_quarter(float unit);

/** Raised-cosine blend of velocity from v0 toward v1 over phase phi in [0,1]. */
float planner_sine_vel(float v0, float v1, float phi);

/**
 * Advance phase for a sine ramp given peak accel a and |v1-v0|.
 * Returns new phi clamped to [0,1]. dt_s > 0.
 * Float matches the filler, which advances phi with a cached 1/T.
 */
float planner_sine_advance_phi(float phi, float v0, float v1, float accel_mm_s2, float dt_s);

/** Steps to bleed |vel| via sine stop-distance law (ceil); 0 if already stopped. */
int planner_stop_rem_steps(float vel_mm_s, float accel_mm_s2, float steps_per_unit);

/**
 * True when a new target sign opposes travel and speed is still significant —
 * fill path must decelerate in place before reversing.
 */
bool planner_needs_reverse_decel(int last_sign, int target_sign, float vel_mm_s);

#ifdef __cplusplus
}
#endif
