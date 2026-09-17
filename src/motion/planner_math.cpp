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

float planner_vmax_triangle(float dist_mm, float accel_mm_s2, float decel_mm_s2) {
  if (dist_mm <= 0.0f) {
    return 0.0f;
  }
  float aa = accel_mm_s2 < 1e-3f ? 1e-3f : accel_mm_s2;
  float ad = decel_mm_s2 < 1e-3f ? 1e-3f : decel_mm_s2;
  return sqrtf(4.0f * dist_mm / ((float)M_PI * (1.0f / aa + 1.0f / ad)));
}

int planner_pack_n_min(float step_hz, int remaining_steps, int pending_steps,
                       int pack_min_hz) {
  if (remaining_steps <= 0) {
    return 0;
  }
  if (pack_min_hz < 1) {
    pack_min_hz = PLANNER_PACK_MIN_HZ;
  }
  if (step_hz < (float)pack_min_hz) {
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

int planner_pack_n(float step_hz, int remaining_steps, int pending_steps) {
  return planner_pack_n_min(step_hz, remaining_steps, pending_steps, PLANNER_PACK_MIN_HZ);
}

uint32_t planner_hz_to_delay(float step_hz, uint32_t sysclk_hz, uint32_t fixed_cycles) {
  if (step_hz < 1.0f) {
    step_hz = 1.0f;
  }
  if (sysclk_hz == 0) {
    sysclk_hz = 50000000u; /* PIO STEP SM clock (clk_sys / clkdiv) */
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

#define PLANNER_SIN_LUT_N 64
/* sin(k * π/2 / N) for k = 0..N. Endpoints are exact 0 and 1. */
static const float k_sin_lut[PLANNER_SIN_LUT_N + 1] = {
    0.000000000e+00f, 2.454122852e-02f, 4.906767433e-02f, 7.356456360e-02f,
    9.801714033e-02f, 1.224106752e-01f, 1.467304745e-01f, 1.709618888e-01f,
    1.950903220e-01f, 2.191012402e-01f, 2.429801799e-01f, 2.667127575e-01f,
    2.902846773e-01f, 3.136817404e-01f, 3.368898534e-01f, 3.598950365e-01f,
    3.826834324e-01f, 4.052413140e-01f, 4.275550934e-01f, 4.496113297e-01f,
    4.713967368e-01f, 4.928981922e-01f, 5.141027442e-01f, 5.349976199e-01f,
    5.555702330e-01f, 5.758081914e-01f, 5.956993045e-01f, 6.152315906e-01f,
    6.343932842e-01f, 6.531728430e-01f, 6.715589548e-01f, 6.895405447e-01f,
    7.071067812e-01f, 7.242470830e-01f, 7.409511254e-01f, 7.572088465e-01f,
    7.730104534e-01f, 7.883464276e-01f, 8.032075315e-01f, 8.175848132e-01f,
    8.314696123e-01f, 8.448535652e-01f, 8.577286100e-01f, 8.700869911e-01f,
    8.819212643e-01f, 8.932243012e-01f, 9.039892931e-01f, 9.142097557e-01f,
    9.238795325e-01f, 9.329927988e-01f, 9.415440652e-01f, 9.495281806e-01f,
    9.569403357e-01f, 9.637760658e-01f, 9.700312532e-01f, 9.757021300e-01f,
    9.807852804e-01f, 9.852776424e-01f, 9.891765100e-01f, 9.924795346e-01f,
    9.951847267e-01f, 9.972904567e-01f, 9.987954562e-01f, 9.996988187e-01f,
    1.000000000e+00f,
};

/* x in [0, π/2]. */
static float planner_sin_quarter(float x) {
  static const float scale = (float)PLANNER_SIN_LUT_N * (2.0f / (float)M_PI);
  float t = x * scale;
  if (t <= 0.0f) {
    return 0.0f;
  }
  if (t >= (float)PLANNER_SIN_LUT_N) {
    return 1.0f;
  }
  int i = (int)t;
  float a = k_sin_lut[i];
  return a + (k_sin_lut[i + 1] - a) * (t - (float)i);
}

float planner_sinf(float x) {
  float ax = x;
  float sign = 1.0f;
  if (ax < 0.0f) {
    ax = -ax;
    sign = -1.0f;
  }
  /* Planner args stay in [0, π]; one 2π peel covers a slight overshoot. */
  if (ax > (float)(2.0 * M_PI)) {
    ax -= (float)(2.0 * M_PI);
  }
  if (ax > (float)M_PI) {
    ax -= (float)M_PI;
    sign = -sign;
  }
  if (ax > (float)(0.5 * M_PI)) {
    ax = (float)M_PI - ax;
  }
  return sign * planner_sin_quarter(ax);
}

float planner_cosf(float x) {
  return planner_sinf((float)(0.5 * M_PI) - x);
}

float planner_sine_vel(float v0, float v1, float phi) {
  if (phi <= 0.0f) {
    return v0;
  }
  if (phi >= 1.0f) {
    return v1;
  }
  /* Raised cosine: v = v0 + (v1-v0) * 0.5 * (1 - cos(pi*phi)) */
  float w = 0.5f * (1.0f - planner_cosf((float)M_PI * phi));
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
