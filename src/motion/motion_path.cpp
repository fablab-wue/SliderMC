#include "motion_path.h"
#include "config_store.h"
#include "config_defaults.h"
#include "motion_api.h"

#include <math.h>
#include <string.h>

#ifndef HOST_TEST
#include "planner.h"
#include "planner_math.h"
#include "pio_step.h"
#endif

/* Scan bound per fill call: caps how many consecutive zero-distance (stand-
 * still) slices are skipped in one call so the highest-priority feed task
 * cannot be starved by a long silent stretch of the buffer. */
#define PATH_FILL_SCAN_BUDGET 256

static int16_t g_path_buf[PATH_BUFFER_MAX];
static uint32_t g_path_count;
static uint32_t g_path_play_index;
static bool g_path_active;

static uint32_t g_path_slice_us_active; /* snapshot of session slice at PG time */
static double g_step_err;
static double g_time_err;
static double g_gap_cycles;

static int64_t g_path_pos_steps;
static float g_path_last_vel_mm_s;

static bool g_slice_in_progress;
static int32_t g_slice_steps_left;
static int g_slice_sign;
static uint32_t g_slice_delay_cycles;

void motion_path_init(void) {
  memset(g_path_buf, 0, sizeof(g_path_buf));
  g_path_count = 0;
  g_path_play_index = 0;
  g_path_active = false;
  g_path_slice_us_active = 0;
  g_step_err = 0.0;
  g_time_err = 0.0;
  g_gap_cycles = 0.0;
  g_path_pos_steps = 0;
  g_path_last_vel_mm_s = 0.0f;
  g_slice_in_progress = false;
  g_slice_steps_left = 0;
  g_slice_sign = 0;
  g_slice_delay_cycles = 0;
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

bool motion_path_add(int16_t distance_um) {
  /* Allowed while active: live-move streaming appends ahead of playback. */
  if ((int)g_path_count >= path_buffer_limit()) {
    return false;
  }
  /* Write the sample before publishing the new count (feed task reads count
     as the visibility fence for the buffer entry it guards). */
  g_path_buf[g_path_count] = distance_um;
  ++g_path_count;
  return true;
}

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

int32_t motion_path_diffuse_steps(int16_t distance_um, float steps_per_mm, double *step_err) {
  if (distance_um == 0) {
    return 0;
  }
  double mm = (double)distance_um / 1000.0;
  double steps_f = mm * (double)steps_per_mm + *step_err;
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
  float spmm = config_get()->steps_per_mm;
  if (spmm < 1e-3f) {
    spmm = 1.0f;
  }
  out->pos_mm = (float)g_path_pos_steps / spmm;
  out->vel_mm_s = g_path_last_vel_mm_s;
  out->has_target = false;
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
  float spmm = config_get()->steps_per_mm;
  if (spmm < 1e-3f) {
    spmm = 1.0f;
  }
  g_path_pos_steps = (int64_t)lroundf(st.pos_mm * spmm);
  g_path_play_index = 0;
  g_step_err = 0.0;
  g_time_err = 0.0;
  g_gap_cycles = 0.0;
  g_path_last_vel_mm_s = 0.0f;
  g_slice_in_progress = false;
  g_slice_steps_left = 0;
  g_path_slice_us_active = (uint32_t)session_get()->path_slice_us;
  if (g_path_slice_us_active < (uint32_t)PATH_SLICE_US_MIN) {
    g_path_slice_us_active = (uint32_t)PATH_SLICE_US_MIN;
  }
  g_path_active = true;
  pio_step_clear_stall();
  pio_step_start();
  pio_step_kick_feed();
  return true;
}

void motion_path_abort_to_planner(void) {
  if (!g_path_active) {
    return;
  }
  planner_takeover_from_path(g_path_pos_steps, g_path_last_vel_mm_s);
  g_path_active = false;
  g_slice_in_progress = false;
}

int motion_path_fill_fifo(void) {
  int emitted = 0;
  unsigned scan_budget = PATH_FILL_SCAN_BUDGET;
  uint32_t sysclk = pio_step_sysclk_hz();
  float spmm = config_get()->steps_per_mm;
  if (spmm < 1e-3f) {
    spmm = 1.0f;
  }

  while (g_path_active && pio_step_tx_room() > 0 && scan_budget--) {
    if (!g_slice_in_progress) {
      if (g_path_play_index >= g_path_count) {
        motion_path_abort_to_planner();
        planner_request_stop();
        break;
      }
      int16_t distance_um = g_path_buf[g_path_play_index++];
      int32_t steps = motion_path_diffuse_steps(distance_um, spmm, &g_step_err);
      uint32_t cycles = motion_path_diffuse_cycles(g_path_slice_us_active, sysclk, &g_time_err);

      if (steps == 0) {
        g_gap_cycles += (double)cycles;
        g_path_last_vel_mm_s = 0.0f;
        continue;
      }

      double total_cycles = (double)cycles + g_gap_cycles;
      g_gap_cycles = 0.0;
      int32_t n_total = steps < 0 ? -steps : steps;
      double period_cycles = total_cycles / (double)n_total;
      double step_hz = (double)sysclk / period_cycles;
      g_slice_delay_cycles = planner_hz_to_delay((float)step_hz, sysclk, PIO_STEP_PERIOD_FIXED);
      g_slice_sign = steps > 0 ? 1 : -1;
      g_slice_steps_left = n_total;
      g_path_last_vel_mm_s = (float)(step_hz / (double)spmm) * (float)g_slice_sign;
    }

    int n = g_slice_steps_left;
    if (n > PLANNER_PACK_MAX) {
      n = PLANNER_PACK_MAX;
    }
    pio_step_set_dir(g_slice_sign);
    if (!pio_step_put_word(g_slice_delay_cycles, (uint8_t)n)) {
      break; /* TX full; resume this slice on the next call */
    }
    g_path_pos_steps += (int64_t)n * (int64_t)g_slice_sign;
    g_slice_steps_left -= n;
    if (g_slice_steps_left <= 0) {
      g_slice_in_progress = false;
    } else {
      g_slice_in_progress = true;
    }
    ++emitted;
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
