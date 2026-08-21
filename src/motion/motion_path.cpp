#include "motion_path.h"
#include "config_store.h"
#include "config_defaults.h"
#include "motion_api.h"
#include "axis_hw.h"

#include <math.h>
#include <string.h>

#ifndef HOST_TEST
#include "planner.h"
#include "planner_math.h"
#include "pio_step.h"
#endif

#define PATH_FILL_SCAN_BUDGET 256

static int16_t g_path_buf[2][PATH_BUFFER_MAX];
static uint32_t g_path_count;
static uint32_t g_path_play_index;
static bool g_path_active;

static uint32_t g_path_slice_us_active;
static double g_step_err[2];
static double g_time_err;
static double g_gap_cycles;

static int64_t g_path_pos_steps[2];
static float g_path_last_vel_mm_s[2];

static bool g_slice_in_progress;
static int32_t g_slice_steps_left[2];
static int g_slice_sign[2];
static uint32_t g_slice_delay_cycles[2];
static bool g_slice_has_steps[2];

void motion_path_init(void) {
  memset(g_path_buf, 0, sizeof(g_path_buf));
  g_path_count = 0;
  g_path_play_index = 0;
  g_path_active = false;
  g_path_slice_us_active = 0;
  g_step_err[0] = g_step_err[1] = 0.0;
  g_time_err = 0.0;
  g_gap_cycles = 0.0;
  g_path_pos_steps[0] = g_path_pos_steps[1] = 0;
  g_path_last_vel_mm_s[0] = g_path_last_vel_mm_s[1] = 0.0f;
  g_slice_in_progress = false;
  g_slice_steps_left[0] = g_slice_steps_left[1] = 0;
  g_slice_sign[0] = g_slice_sign[1] = 0;
  g_slice_delay_cycles[0] = g_slice_delay_cycles[1] = 0;
  g_slice_has_steps[0] = g_slice_has_steps[1] = false;
}

static int path_buffer_limit(void) {
  int limit = config_get()->path_buffer_size;
  if (limit < 1) {
    limit = 1;
  }
  if (limit > PATH_BUFFER_MAX) {
    limit = PATH_BUFFER_MAX;
  }
  return limit;
}

bool motion_path_clear(void) {
  if (g_path_active) {
    return false;
  }
  g_path_count = 0;
  g_path_play_index = 0;
  return true;
}

bool motion_path_add2(int16_t a_um, int16_t b_um) {
  if ((int)g_path_count >= path_buffer_limit()) {
    return false;
  }
  g_path_buf[0][g_path_count] = a_um;
  g_path_buf[1][g_path_count] = b_um;
  ++g_path_count;
  return true;
}

bool motion_path_add(int16_t distance_um) { return motion_path_add2(distance_um, 0); }

uint32_t motion_path_count(void) { return g_path_count; }

bool motion_path_set_slice_us(uint32_t us) {
  if (g_path_active || us < (uint32_t)PATH_SLICE_US_MIN) {
    return false;
  }
  session_get()->path_slice_us = (int)us;
  return true;
}

uint32_t motion_path_get_slice_us(void) { return (uint32_t)session_get()->path_slice_us; }

bool motion_path_is_active(void) { return g_path_active; }

int32_t motion_path_diffuse_steps(int16_t distance_um, float steps_per_unit, double *step_err) {
  if (distance_um == 0) {
    return 0;
  }
  double mm = (double)distance_um / 1000.0;
  double steps_f = mm * (double)steps_per_unit + *step_err;
  int32_t steps_i = (int32_t)llround(steps_f);
  *step_err = steps_f - (double)steps_i;
  return steps_i;
}

uint32_t motion_path_diffuse_cycles(uint32_t slice_us, uint32_t sysclk_hz, double *time_err) {
  double cycles_f = (double)slice_us * ((double)sysclk_hz / 1e6) + *time_err;
  uint32_t cycles_i = (uint32_t)llround(cycles_f);
  *time_err = cycles_f - (double)cycles_i;
  return cycles_i;
}

void motion_path_get_status(McStatus *out) {
  if (!out) {
    return;
  }
  memset(out, 0, sizeof(*out));
  out->state = MC_STATE_PATH;
  out->enabled = true;
  out->moving = true;
  float spmm0 = axis_hw_steps_per_unit(0);
  if (spmm0 < 1e-3f) {
    spmm0 = 1.0f;
  }
  out->pos_mm = (float)g_path_pos_steps[0] / spmm0;
  out->pos_mm_2 = 0.0f;
  if (config_axis2_enabled()) {
    float spmm1 = axis_hw_steps_per_unit(1);
    if (spmm1 < 1e-3f) {
      spmm1 = 1.0f;
    }
    out->pos_mm_2 = (float)g_path_pos_steps[1] / spmm1;
  }
  out->vel_mm_s = g_path_last_vel_mm_s[0];
  out->vel_mm_s_2 = 0.0f;
  out->acc_mm_s2 = 0.0f;
  out->acc_mm_s2_2 = 0.0f;
  out->target_mm = out->pos_mm;
  out->target_mm_2 = out->pos_mm_2;
  out->has_target = false;
  if (config_axis2_enabled()) {
    out->vel_mm_s_2 = g_path_last_vel_mm_s[1];
  }
}

#ifndef HOST_TEST

bool motion_path_go(void) {
  if (g_path_active || g_path_count == 0) {
    return false;
  }
  McStatus st;
  motion_get_status(&st);
  if (!st.enabled) {
    return false;
  }
  float spmm0 = axis_hw_steps_per_unit(0);
  if (spmm0 < 1e-3f) {
    spmm0 = 1.0f;
  }
  g_path_pos_steps[0] = (int64_t)lroundf(st.pos_mm * spmm0);
  g_path_pos_steps[1] = 0;
  if (config_axis2_enabled()) {
    float spmm1 = axis_hw_steps_per_unit(1);
    if (spmm1 < 1e-3f) {
      spmm1 = 1.0f;
    }
    g_path_pos_steps[1] = (int64_t)lroundf(st.pos_mm_2 * spmm1);
  }
  g_path_play_index = 0;
  g_step_err[0] = g_step_err[1] = 0.0;
  g_time_err = 0.0;
  g_gap_cycles = 0.0;
  g_path_last_vel_mm_s[0] = g_path_last_vel_mm_s[1] = 0.0f;
  g_slice_in_progress = false;
  g_slice_steps_left[0] = g_slice_steps_left[1] = 0;
  g_path_slice_us_active = (uint32_t)session_get()->path_slice_us;
  if (g_path_slice_us_active < (uint32_t)PATH_SLICE_US_MIN) {
    g_path_slice_us_active = (uint32_t)PATH_SLICE_US_MIN;
  }
  g_path_active = true;
  pio_step_clear_stall(0);
  pio_step_start(0);
  if (config_axis2_enabled()) {
    pio_step_clear_stall(1);
    pio_step_start(1);
  }
  pio_step_kick_feed();
  return true;
}

void motion_path_abort_to_planner(void) {
  if (!g_path_active) {
    return;
  }
  planner_takeover_from_path(0, g_path_pos_steps[0], g_path_last_vel_mm_s[0]);
  if (config_axis2_enabled()) {
    planner_takeover_from_path(1, g_path_pos_steps[1], g_path_last_vel_mm_s[1]);
  }
  g_path_active = false;
  g_slice_in_progress = false;
}

int motion_path_fill_fifo(void) {
  int emitted = 0;
  unsigned scan_budget = PATH_FILL_SCAN_BUDGET;
  uint32_t sysclk = pio_step_sysclk_hz();
  const bool ax2 = config_axis2_enabled();
  float spmm[2];
  spmm[0] = axis_hw_steps_per_unit(0);
  if (spmm[0] < 1e-3f) {
    spmm[0] = 1.0f;
  }
  spmm[1] = ax2 ? axis_hw_steps_per_unit(1) : 1.0f;
  if (spmm[1] < 1e-3f) {
    spmm[1] = 1.0f;
  }

  while (g_path_active && scan_budget--) {
    /* Lockstep: both SMs must have room when axis2 is active. */
    if (pio_step_tx_room(0) == 0) {
      break;
    }
    if (ax2 && pio_step_tx_room(1) == 0) {
      break;
    }

    if (!g_slice_in_progress) {
      if (g_path_play_index >= g_path_count) {
        motion_path_abort_to_planner();
        planner_request_stop();
        break;
      }
      int16_t d0 = g_path_buf[0][g_path_play_index];
      int16_t d1 = g_path_buf[1][g_path_play_index];
      ++g_path_play_index;

      int32_t steps0 = motion_path_diffuse_steps(d0, spmm[0], &g_step_err[0]);
      int32_t steps1 = ax2 ? motion_path_diffuse_steps(d1, spmm[1], &g_step_err[1]) : 0;
      uint32_t cycles = motion_path_diffuse_cycles(g_path_slice_us_active, sysclk, &g_time_err);

      if (steps0 == 0 && steps1 == 0) {
        g_gap_cycles += (double)cycles;
        g_path_last_vel_mm_s[0] = 0.0f;
        g_path_last_vel_mm_s[1] = 0.0f;
        continue;
      }

      double total_cycles = (double)cycles + g_gap_cycles;
      g_gap_cycles = 0.0;

      for (int a = 0; a < (ax2 ? 2 : 1); ++a) {
        int32_t steps = (a == 0) ? steps0 : steps1;
        if (steps == 0) {
          g_slice_has_steps[a] = false;
          g_slice_steps_left[a] = 0;
          g_path_last_vel_mm_s[a] = 0.0f;
          continue;
        }
        int32_t n_total = steps < 0 ? -steps : steps;
        double period_cycles = total_cycles / (double)n_total;
        double step_hz = (double)sysclk / period_cycles;
        g_slice_delay_cycles[a] = planner_hz_to_delay((float)step_hz, sysclk, PIO_STEP_PERIOD_FIXED);
        g_slice_sign[a] = steps > 0 ? 1 : -1;
        g_slice_steps_left[a] = n_total;
        g_slice_has_steps[a] = true;
        g_path_last_vel_mm_s[a] = (float)(step_hz / (double)spmm[a]) * (float)g_slice_sign[a];
      }
      g_slice_in_progress = true;
    }

    bool progressed = false;
    for (int a = 0; a < (ax2 ? 2 : 1); ++a) {
      if (!g_slice_has_steps[a] || g_slice_steps_left[a] <= 0) {
        continue;
      }
      int n = g_slice_steps_left[a];
      if (n > PLANNER_PACK_MAX) {
        n = PLANNER_PACK_MAX;
      }
      pio_step_set_dir(a, g_slice_sign[a]);
      if (!pio_step_put_word(a, g_slice_delay_cycles[a], (uint8_t)n)) {
        /* TX full — keep slice in progress */
        g_slice_in_progress = true;
        return emitted;
      }
      g_path_pos_steps[a] += (int64_t)n * (int64_t)g_slice_sign[a];
      g_slice_steps_left[a] -= n;
      progressed = true;
      ++emitted;
    }

    bool done = true;
    for (int a = 0; a < (ax2 ? 2 : 1); ++a) {
      if (g_slice_has_steps[a] && g_slice_steps_left[a] > 0) {
        done = false;
      }
    }
    if (done) {
      g_slice_in_progress = false;
    }
    if (!progressed && g_slice_in_progress) {
      break;
    }
  }
  return emitted;
}

#else /* HOST_TEST */

bool motion_path_go(void) {
  if (g_path_active || g_path_count == 0) {
    return false;
  }
  McStatus st;
  motion_get_status(&st);
  if (!st.enabled) {
    return false;
  }
  g_path_play_index = 0;
  g_path_active = true;
  return true;
}

void motion_path_abort_to_planner(void) {
  g_path_active = false;
  g_slice_in_progress = false;
}

int motion_path_fill_fifo(void) { return 0; }

#endif /* HOST_TEST */
