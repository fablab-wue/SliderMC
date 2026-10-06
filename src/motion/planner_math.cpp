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
  /* Word and horizon are whole milliseconds (2 and 6). Integer Hz avoids a
   * soft-float multiply on every one-pulse word. */
  uint32_t hz = (step_hz >= 1.0f) ? (uint32_t)step_hz : 1u;
  uint32_t word_ms = (uint32_t)PLANNER_FIFO_WORD_MS;
  uint32_t horizon_ms = (uint32_t)PLANNER_FIFO_HORIZON_MS;
  if (word_ms < 1u) {
    word_ms = 1u;
  }
  if (horizon_ms < word_ms) {
    horizon_ms = word_ms;
  }
  int horizon_steps = (int)(hz * horizon_ms / 1000u);
  if (horizon_steps < 1) {
    horizon_steps = 1;
  }
  int room = horizon_steps - pending_steps;
  if (room < 1) {
    return 0;
  }
  int n = (int)(hz * word_ms / 1000u);
  if (n < 1) {
    n = 1;
  }
  if (n > room) {
    n = room;
  }
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
  if (sysclk_hz == 0) {
    sysclk_hz = 50000000u; /* PIO STEP SM clock (clk_sys / clkdiv) */
  }
  /* Integer Hz is finer than the 2% refinement gate, and the divide is a
   * machine instruction instead of a soft-float libcall on RP2040. */
  uint32_t hz = (step_hz >= 1.0f) ? (uint32_t)step_hz : 1u;
  uint32_t period = sysclk_hz / hz;
  if (period <= fixed_cycles) {
    return 1u;
  }
  uint32_t delay = period - fixed_cycles;
  if (delay > 0x03FFFFFFu) {
    delay = 0x03FFFFFFu;
  }
  if (delay < 1u) {
    delay = 1u;
  }
  return delay;
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

/* t is the table index in [0, N]. N maps to sin(π/2) = 1. */
static float planner_sin_quarter_t(float t) {
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

/* x in [0, π/2]. */
static float planner_sin_quarter(float x) {
  static const float scale = (float)PLANNER_SIN_LUT_N * (2.0f / (float)M_PI);
  return planner_sin_quarter_t(x * scale);
}

float planner_sinf_unit_quarter(float unit) {
  if (unit <= 0.0f) {
    return 0.0f;
  }
  if (unit >= 1.0f) {
    return 1.0f;
  }
  /* unit 0 → sin 0, unit 1 → sin(π/2). Same samples as the quarter LUT. */
  return planner_sin_quarter_t(unit * (float)PLANNER_SIN_LUT_N);
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

/* 0.5 * (1 - cos(π * k/N)) for k = 0..N. phi is already in [0, 1]. */
static const float k_raised_cos[PLANNER_SIN_LUT_N + 1] = {
    0.000000000e+00f, 6.022718974e-04f, 2.407636664e-03f, 5.411745018e-03f,
    9.607359798e-03f, 1.498437340e-02f, 2.152983213e-02f, 2.922796741e-02f,
    3.806023374e-02f, 4.800535344e-02f, 5.903936783e-02f, 7.113569500e-02f,
    8.426519385e-02f, 9.839623426e-02f, 1.134947733e-01f, 1.295244373e-01f,
    1.464466094e-01f, 1.642205226e-01f, 1.828033579e-01f, 2.021503478e-01f,
    2.222148835e-01f, 2.429486279e-01f, 2.643016316e-01f, 2.862224533e-01f,
    3.086582838e-01f, 3.315550733e-01f, 3.548576614e-01f, 3.785099100e-01f,
    4.024548390e-01f, 4.266347628e-01f, 4.509914298e-01f, 4.754661628e-01f,
    5.000000000e-01f, 5.245338372e-01f, 5.490085702e-01f, 5.733652372e-01f,
    5.975451610e-01f, 6.214900900e-01f, 6.451423386e-01f, 6.684449267e-01f,
    6.913417162e-01f, 7.137775467e-01f, 7.356983684e-01f, 7.570513721e-01f,
    7.777851165e-01f, 7.978496522e-01f, 8.171966421e-01f, 8.357794774e-01f,
    8.535533906e-01f, 8.704755627e-01f, 8.865052267e-01f, 9.016037657e-01f,
    9.157348062e-01f, 9.288643050e-01f, 9.409606322e-01f, 9.519946466e-01f,
    9.619397663e-01f, 9.707720326e-01f, 9.784701679e-01f, 9.850156266e-01f,
    9.903926402e-01f, 9.945882550e-01f, 9.975923633e-01f, 9.993977281e-01f,
    1.000000000e+00f,
};

static float planner_raised_cosine(float phi) {
  if (phi <= 0.0f) {
    return 0.0f;
  }
  if (phi >= 1.0f) {
    return 1.0f;
  }
  float t = phi * (float)PLANNER_SIN_LUT_N;
  if (t >= (float)PLANNER_SIN_LUT_N) {
    return 1.0f;
  }
  int i = (int)t;
  float a = k_raised_cos[i];
  return a + (k_raised_cos[i + 1] - a) * (t - (float)i);
}

float planner_sine_vel(float v0, float v1, float phi) {
  if (phi <= 0.0f) {
    return v0;
  }
  if (phi >= 1.0f) {
    return v1;
  }
  /* Raised cosine: v = v0 + (v1-v0) * 0.5 * (1 - cos(pi*phi)) */
  return v0 + (v1 - v0) * planner_raised_cosine(phi);
}

float planner_sine_advance_phi(float phi, float v0, float v1, float accel_mm_s2, float dt_s) {
  float dv = fabsf(v1 - v0);
  if (dv < 1e-6f || accel_mm_s2 < 1e-6f || dt_s <= 0.0f) {
    return 1.0f;
  }
  /* Duration of half-sine accel: T = pi * |dv| / (2 a) */
  float T = (float)M_PI * dv / (2.0f * accel_mm_s2);
  if (T < 1e-6f) {
    return 1.0f;
  }
  phi += dt_s / T;
  if (phi > 1.0f) {
    phi = 1.0f;
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
  float d_steps = fabsf(d_stop) * spu;
  int rem = (int)(d_steps + 0.999f);
  if (rem < 1) {
    rem = 1;
  }
  return rem;
}

bool planner_needs_reverse_decel(int last_sign, int target_sign, float vel_mm_s) {
  return last_sign != 0 && target_sign != 0 && target_sign != last_sign &&
         fabsf(vel_mm_s) > 0.05f;
}
