#include "planner_math.h"

#include <math.h>
#include <stdbool.h>

float planner_vmax_for_distance(float dist_mm, float accel_mm_s2) {
  if (dist_mm <= 0.0f) {
    return 0.0f;
  }
  float a = accel_mm_s2;
  if (a < 1e-3f) {
    a = 1e-3f;
  }
  return sqrtf(4.0f * a * dist_mm / (float)M_PI);
}

int planner_pack_n(float step_hz, int remaining_steps, int pending_steps) {
  if (remaining_steps <= 0) {
    return 0;
  }
  if (step_hz < (float)PLANNER_PACK_MIN_HZ) {
    return 1;
  }
  float budget_ms = PLANNER_FIFO_TIME_BUDGET_MS;
  if (budget_ms < 0.5f) {
    budget_ms = 0.5f;
  }
  int budget_steps = (int)(step_hz * budget_ms / 1000.0f);
  if (budget_steps < 1) {
    budget_steps = 1;
  }
  int room = budget_steps - pending_steps;
  if (room < 1) {
    return 0;
  }
  int n = room;
  if (n > PLANNER_PACK_MAX) {
    n = PLANNER_PACK_MAX;
  }
  if (n > remaining_steps) {
    n = remaining_steps;
  }
  /* Keep staircase samples near target (~20% of remaining per word). */
  int max_n = remaining_steps / 5;
  if (max_n < 1) {
    max_n = 1;
  }
  if (n > max_n) {
    n = max_n;
  }
  return n;
}

uint32_t planner_hz_to_delay(float step_hz, uint32_t sysclk_hz, uint32_t fixed_cycles) {
  if (step_hz < 1.0f) {
    step_hz = 1.0f;
  }
  if (sysclk_hz == 0) {
    sysclk_hz = 50000000u;  // effective PIO SM clock after clkdiv 2.5
  }
  float period = (float)sysclk_hz / step_hz;
  float delay_f = period - (float)fixed_cycles;
  if (delay_f < 1.0f) {
    delay_f = 1.0f;
  }
  if (delay_f > (float)0x03FFFFFFu) {
    delay_f = (float)0x03FFFFFFu;
  }
  return (uint32_t)delay_f;
}

float planner_max_step_hz(uint32_t sysclk_hz, uint32_t fixed_cycles) {
  if (sysclk_hz == 0) {
    sysclk_hz = 50000000u;  // effective PIO SM clock at 50 MHz
  }
  uint32_t min_period = fixed_cycles + 1u;
  if (min_period == 0) {
    return 0.0f;
  }
  return (float)sysclk_hz / (float)min_period;
}

float planner_sine_vel(float v0, float v1, float phi) {
  if (phi <= 0.0f) {
    return v0;
  }
  if (phi >= 1.0f) {
    return v1;
  }
  /* Raised cosine: v = v0 + (v1-v0) * 0.5 * (1 - cos(pi*phi)) */
  float w = 0.5f * (1.0f - cosf((float)M_PI * phi));
  return v0 + (v1 - v0) * w;
}

double planner_sine_advance_phi(double phi, float v0, float v1, float accel_mm_s2,
                               double dt_s) {
  double dv = fabs((double)v1 - (double)v0);
  if (dv < 1e-6 || accel_mm_s2 < 1e-6f || dt_s <= 0.0) {
    return 1.0;
  }
  /* Duration of half-sine accel: T = pi * |dv| / (2 a) */
  double T = (double)M_PI * dv / (2.0 * (double)accel_mm_s2);
  if (T < 1e-6) {
    return 1.0;
  }
  phi += dt_s / T;
  if (phi > 1.0) {
    phi = 1.0;
  }
  return phi;
}

int planner_stop_rem_steps(float vel_mm_s, float accel_mm_s2, float steps_per_unit) {
  if (fabsf(vel_mm_s) < 0.01f) {
    return 0;
  }
  float a = accel_mm_s2 > 1e-3f ? accel_mm_s2 : 1e-3f;
  float spu = steps_per_unit > 1e-3f ? steps_per_unit : 1.0f;
  float d_stop = (float)M_PI * vel_mm_s * vel_mm_s / (4.0f * a);
  int rem = (int)ceilf(fabsf(d_stop) * spu);
  if (rem < 1) {
    rem = 1;
  }
  return rem;
}

bool planner_needs_reverse_decel(int last_sign, int target_sign, float vel_mm_s) {
  return last_sign != 0 && target_sign != 0 && target_sign != last_sign &&
         fabsf(vel_mm_s) > 0.05f;
}
