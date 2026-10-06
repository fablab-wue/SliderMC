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
#define PATH_AXES MC_CH_MAX

static int16_t g_path_pool[PATH_POOL_SAMPLES];
static uint32_t g_path_count;
static uint32_t g_path_play_index;
static uint32_t g_path_play_last; /* inclusive last sample */
static int g_path_dir;           /* +1 forward, -1 reverse */
static bool g_path_range_done;
static bool g_path_active;

static uint32_t g_path_slice_us_active;
/* Leftover steps in Q24 (2^24 units = 1 step), and leftover SM cycles in
 * millionths of a cycle. Whole gap cycles wait for the next moving slice. */
#define PATH_STEP_SCALE (1 << 24)
#define PATH_CYCLE_SCALE 1000000ll
static int64_t g_step_rem[PATH_AXES];
static int64_t g_time_rem;
static uint64_t g_gap_cycles;

static int64_t g_path_pos_steps[PATH_AXES];
static float g_path_last_vel_mm_s[PATH_AXES];

static bool g_slice_in_progress;
static bool g_slice_drain_wait;
static bool g_slice_had_steps[PATH_AXES];
static int32_t g_slice_steps_left[PATH_AXES];
static int g_slice_sign[PATH_AXES];
static uint32_t g_slice_delay_cycles[PATH_AXES];
static bool g_slice_has_steps[PATH_AXES];

static int path_naxes(void) { return config_axis_count(); }

static int path_stride(void) { return PATH_POOL_SAMPLES / path_naxes(); }

static int16_t *path_col(int axis) { return &g_path_pool[axis * path_stride()]; }

void motion_path_init(void) {
  memset(g_path_pool, 0, sizeof(g_path_pool));
  g_path_count = 0;
  g_path_play_index = 0;
  g_path_play_last = 0;
  g_path_dir = 1;
  g_path_range_done = false;
  g_path_active = false;
  g_path_slice_us_active = 0;
  for (int a = 0; a < PATH_AXES; ++a) {
    g_step_rem[a] = 0;
    g_path_pos_steps[a] = 0;
    g_path_last_vel_mm_s[a] = 0.0f;
    g_slice_steps_left[a] = 0;
    g_slice_sign[a] = 0;
    g_slice_delay_cycles[a] = 0;
    g_slice_has_steps[a] = false;
    g_slice_had_steps[a] = false;
  }
  g_time_rem = 0;
  g_gap_cycles = 0;
  g_slice_in_progress = false;
  g_slice_drain_wait = false;
}

static int path_buffer_limit(void) {
  int limit = config_get()->path_buffer_size;
  int cap = config_path_per_axis_max();
  if (limit < 1) {
    limit = 1;
  }
  if (limit > cap) {
    limit = cap;
  }
  return limit;
}

bool motion_path_clear(void) {
  if (g_path_active) {
    return false;
  }
  g_path_count = 0;
  g_path_play_index = 0;
  g_path_play_last = 0;
  g_path_dir = 1;
  g_path_range_done = false;
  return true;
}

bool motion_path_addn(const int16_t *samp) {
  if (!samp || (int)g_path_count >= path_buffer_limit()) {
    return false;
  }
  int n = path_naxes();
  for (int a = 0; a < n; ++a) {
    path_col(a)[g_path_count] = samp[a];
  }
  ++g_path_count;
  return true;
}

bool motion_path_add3(int16_t a_um, int16_t b_um, int16_t c_um) {
  int16_t v[MC_CH_MAX] = {0};
  v[0] = a_um;
  v[1] = b_um;
  v[2] = c_um;
  return motion_path_addn(v);
}

bool motion_path_add2(int16_t a_um, int16_t b_um) { return motion_path_add3(a_um, b_um, 0); }

bool motion_path_add(int16_t distance_um) { return motion_path_add3(distance_um, 0, 0); }

uint32_t motion_path_count(void) { return g_path_count; }

uint32_t motion_path_play_index(void) { return g_path_play_index; }

bool motion_path_set_slice_us(uint32_t us) {
  if (g_path_active || us < (uint32_t)PATH_SLICE_US_MIN) {
    return false;
  }
  session_get()->path_slice_us = (int)us;
  return true;
}

uint32_t motion_path_get_slice_us(void) { return (uint32_t)session_get()->path_slice_us; }

bool motion_path_is_active(void) { return g_path_active; }

static bool path_go_common(uint32_t start_index, uint32_t end_index) {
  if (g_path_active || g_path_count == 0) {
    return false;
  }
  if (start_index >= g_path_count || end_index >= g_path_count) {
    return false;
  }
  McStatus st;
  motion_get_status(&st);
  if (!st.enabled) {
    return false;
  }
  g_path_play_index = start_index;
  g_path_play_last = end_index;
  g_path_dir = (start_index <= end_index) ? 1 : -1;
  g_path_range_done = false;
  return true;
}

#ifndef HOST_TEST
static int32_t path_sample_um(int axis, uint32_t index) {
  int32_t um = (int32_t)path_col(axis)[index];
  if (g_path_dir < 0) {
    um = -um;
  }
  return um;
}
#endif

static int64_t steps_per_um_q(float steps_per_unit) {
  double q = (double)steps_per_unit * (double)PATH_STEP_SCALE / 1000.0;
  return (int64_t)llround(q);
}

static int32_t diffuse_steps_q(int32_t distance_um, int64_t per_um_q, int64_t *rem_q) {
  if (distance_um == 0 || !rem_q) {
    return 0;
  }
  int64_t acc = *rem_q + (int64_t)distance_um * per_um_q;
  const int64_t half = (int64_t)PATH_STEP_SCALE / 2;
  int64_t steps = (acc >= 0) ? ((acc + half) / (int64_t)PATH_STEP_SCALE)
                             : -(((-acc) + half) / (int64_t)PATH_STEP_SCALE);
  *rem_q = acc - steps * (int64_t)PATH_STEP_SCALE;
  if (steps > 2147483647ll) {
    steps = 2147483647ll;
  } else if (steps < -2147483647ll) {
    steps = -2147483647ll;
  }
  return (int32_t)steps;
}

static uint32_t diffuse_cycles_rem(uint32_t slice_us, uint32_t sysclk_hz, int64_t *rem) {
  if (!rem) {
    return 0;
  }
  int64_t num = (int64_t)slice_us * (int64_t)sysclk_hz + *rem;
  const int64_t half = PATH_CYCLE_SCALE / 2;
  int64_t cycles = (num >= 0) ? ((num + half) / PATH_CYCLE_SCALE)
                              : -(((-num) + half) / PATH_CYCLE_SCALE);
  *rem = num - cycles * PATH_CYCLE_SCALE;
  if (cycles < 0) {
    cycles = 0;
  } else if (cycles > 0xffffffffll) {
    cycles = 0xffffffffll;
  }
  return (uint32_t)cycles;
}

int32_t motion_path_diffuse_steps(int16_t distance_um, float steps_per_unit, double *step_err) {
  if (!step_err || distance_um == 0) {
    return 0;
  }
  int64_t rem = (int64_t)llround(*step_err * (double)PATH_STEP_SCALE);
  int32_t steps = diffuse_steps_q((int32_t)distance_um, steps_per_um_q(steps_per_unit), &rem);
  *step_err = (double)rem / (double)PATH_STEP_SCALE;
  return steps;
}

uint32_t motion_path_diffuse_cycles(uint32_t slice_us, uint32_t sysclk_hz, double *time_err) {
  if (!time_err) {
    return 0;
  }
  int64_t rem = (int64_t)llround(*time_err * (double)PATH_CYCLE_SCALE);
  uint32_t cycles = diffuse_cycles_rem(slice_us, sysclk_hz, &rem);
  *time_err = (double)rem / (double)PATH_CYCLE_SCALE;
  return cycles;
}

static float path_pos_mm(int axis) {
  int nm = config_motor_count();
  float spmm;
  if (axis < nm) {
    spmm = axis_hw_steps_per_unit(axis);
  } else {
    spmm = 1000.0f; /* millideg per degree */
  }
  if (spmm < 1e-3f) {
    spmm = 1.0f;
  }
  return (float)g_path_pos_steps[axis] / spmm;
}

void motion_path_get_status(McStatus *out) {
  if (!out) {
    return;
  }
  memset(out, 0, sizeof(*out));
  out->state = MC_STATE_PATH;
  out->enabled = true;
  out->moving = true;
  int n = path_naxes();
  out->pos[0] = path_pos_mm(0);
  out->vel[0] = g_path_last_vel_mm_s[0];
  out->target[0] = out->pos[0];
  for (int a = 1; a < n && a < PATH_AXES; ++a) {
    out->pos[a] = path_pos_mm(a);
    out->vel[a] = g_path_last_vel_mm_s[a];
    out->target[a] = out->pos[a];
  }
}

#ifndef HOST_TEST

bool motion_path_go(void) {
  if (g_path_count == 0) {
    return false;
  }
  return motion_path_go_range(0, g_path_count - 1);
}

bool motion_path_go_range(uint32_t start_index, uint32_t end_index) {
  if (!path_go_common(start_index, end_index)) {
    return false;
  }
  motion_end_joy();
  McStatus st;
  motion_get_status(&st);
  int n = path_naxes();
  int nm = config_motor_count();
  for (int a = 0; a < PATH_AXES; ++a) {
    float spmm = 1000.0f;
    if (a < nm) {
      spmm = axis_hw_steps_per_unit(a);
      if (spmm < 1e-3f) {
        spmm = 1.0f;
      }
    }
    g_path_pos_steps[a] = (a < n) ? (int64_t)lroundf(st.pos[a] * spmm) : 0;
    g_step_rem[a] = 0;
    g_path_last_vel_mm_s[a] = 0.0f;
    g_slice_steps_left[a] = 0;
    g_slice_has_steps[a] = false;
    g_slice_had_steps[a] = false;
  }
  g_time_rem = 0;
  g_gap_cycles = 0;
  g_slice_in_progress = false;
  g_slice_drain_wait = false;
  g_path_slice_us_active = (uint32_t)session_get()->path_slice_us;
  if (g_path_slice_us_active < (uint32_t)PATH_SLICE_US_MIN) {
    g_path_slice_us_active = (uint32_t)PATH_SLICE_US_MIN;
  }
  g_path_active = true;
  for (int a = 0; a < nm; ++a) {
    pio_step_clear_stall(a);
  }
  pio_step_kick_feed();
  return true;
}

void motion_path_abort_to_planner(void) {
  if (!g_path_active) {
    return;
  }
  int n = config_motor_count();
  for (int a = 0; a < n; ++a) {
    planner_takeover_from_path(a, g_path_pos_steps[a], g_path_last_vel_mm_s[a]);
  }
  int ns = config_servo_count();
  int nm = n;
  for (int s = 0; s < ns; ++s) {
    planner_path_set_servo(s, path_pos_mm(nm + s), g_path_last_vel_mm_s[nm + s]);
  }
  g_path_active = false;
  g_slice_in_progress = false;
  g_slice_drain_wait = false;
  g_path_range_done = false;
}

int motion_path_fill_fifo(void) {
  int emitted = 0;
  unsigned scan_budget = PATH_FILL_SCAN_BUDGET;
  uint32_t sysclk = pio_step_sysclk_hz();
  const int n = config_motor_count();
  float spmm[PATH_AXES];
  int64_t step_q[PATH_AXES];
  for (int a = 0; a < n; ++a) {
    spmm[a] = axis_hw_steps_per_unit(a);
    if (spmm[a] < 1e-3f) {
      spmm[a] = 1.0f;
    }
    step_q[a] = steps_per_um_q(spmm[a]);
  }
  const int ns_live = config_servo_count();
  int64_t servo_q = steps_per_um_q(1000.0f);
  for (int s = 0; s < ns_live && (n + s) < PATH_AXES; ++s) {
    step_q[n + s] = servo_q;
  }

  while (g_path_active && scan_budget--) {
    if (g_slice_drain_wait) {
      /* Keep a few words queued. Waiting for empty opened a gap the filler
       * could not close before the next burst, which is an underrun on a
       * mixed slice (common once a fourth axis is often idle). */
      bool low_enough = true;
      for (int a = 0; a < n; ++a) {
        if (g_slice_had_steps[a] && pio_step_tx_level(a) > PIO_STEP_START_MIN_LEVEL) {
          low_enough = false;
          break;
        }
      }
      if (!low_enough) {
        break;
      }
      g_slice_drain_wait = false;
      g_slice_in_progress = false;
    }

    if (!g_slice_in_progress) {
      if (g_path_range_done) {
        motion_path_abort_to_planner();
        planner_request_stop();
        break;
      }
      int32_t steps[PATH_AXES];
      bool any_steps = false;
      bool any_servo = false;
      const int ns = config_servo_count();
      for (int a = 0; a < n; ++a) {
        int32_t um = path_sample_um(a, g_path_play_index);
        steps[a] = diffuse_steps_q(um, step_q[a], &g_step_rem[a]);
        if (steps[a] != 0) {
          any_steps = true;
        }
      }
      int32_t servo_steps[SERVO_MAX];
      for (int s = 0; s < ns; ++s) {
        int a = n + s;
        int32_t um = path_sample_um(a, g_path_play_index);
        servo_steps[s] = diffuse_steps_q(um, step_q[a], &g_step_rem[a]);
        if (servo_steps[s] != 0) {
          any_servo = true;
        }
      }
      if (g_path_play_index == g_path_play_last) {
        g_path_range_done = true;
      } else if (g_path_dir > 0) {
        ++g_path_play_index;
      } else {
        --g_path_play_index;
      }
      uint32_t cycles = diffuse_cycles_rem(g_path_slice_us_active, sysclk, &g_time_rem);

      if (!any_steps && !any_servo) {
        g_gap_cycles += cycles;
        for (int a = 0; a < PATH_AXES; ++a) {
          g_path_last_vel_mm_s[a] = 0.0f;
        }
        for (int s = 0; s < ns; ++s) {
          planner_path_set_servo(s, path_pos_mm(n + s), 0.0f);
        }
        continue;
      }

      float slice_s = (float)g_path_slice_us_active * 1e-6f;
      if (slice_s < 1e-9f) {
        slice_s = 1e-9f;
      }
      for (int s = 0; s < ns; ++s) {
        int a = n + s;
        g_path_pos_steps[a] += (int64_t)servo_steps[s];
        float ddeg = (float)servo_steps[s] / 1000.0f;
        float vel = ddeg / slice_s;
        g_path_last_vel_mm_s[a] = vel;
        planner_path_set_servo(s, path_pos_mm(a), vel);
      }

      if (!any_steps) {
        continue;
      }

      uint64_t total_cycles = (uint64_t)cycles + g_gap_cycles;
      g_gap_cycles = 0;
      bool mixed = false;
      for (int a = 0; a < n; ++a) {
        if (steps[a] == 0) {
          mixed = true;
          g_slice_has_steps[a] = false;
          g_slice_had_steps[a] = false;
          g_slice_steps_left[a] = 0;
          g_path_last_vel_mm_s[a] = 0.0f;
          continue;
        }
        int32_t n_total = steps[a] < 0 ? -steps[a] : steps[a];
        uint32_t period = (n_total > 0) ? (uint32_t)(total_cycles / (uint64_t)n_total) : 0u;
        if (period <= PIO_STEP_PERIOD_FIXED) {
          g_slice_delay_cycles[a] = 1u;
        } else {
          uint32_t delay = period - PIO_STEP_PERIOD_FIXED;
          g_slice_delay_cycles[a] = (delay > 0x03FFFFFFu) ? 0x03FFFFFFu : delay;
        }
        g_slice_sign[a] = steps[a] > 0 ? 1 : -1;
        g_slice_steps_left[a] = n_total;
        g_slice_has_steps[a] = true;
        g_slice_had_steps[a] = true;
        float step_hz = (period > 0u) ? ((float)sysclk / (float)period) : 0.0f;
        g_path_last_vel_mm_s[a] = (step_hz / spmm[a]) * (float)g_slice_sign[a];
      }
      (void)mixed;
      g_slice_in_progress = true;
    }

    bool progressed = false;
    for (int a = 0; a < n; ++a) {
      if (!g_slice_has_steps[a] || g_slice_steps_left[a] <= 0) {
        continue;
      }
      if (pio_step_tx_room(a) == 0) {
        continue;
      }
      int nput = g_slice_steps_left[a];
      if (nput > PLANNER_PACK_MAX) {
        nput = PLANNER_PACK_MAX;
      }
      pio_step_set_dir(a, g_slice_sign[a]);
      if (!pio_step_put_word(a, g_slice_delay_cycles[a], (uint8_t)nput)) {
        continue;
      }
      pio_step_start(a);
      g_path_pos_steps[a] += (int64_t)nput * (int64_t)g_slice_sign[a];
      g_slice_steps_left[a] -= nput;
      progressed = true;
      ++emitted;
    }

    bool queued = true;
    bool any_idle = false;
    for (int a = 0; a < n; ++a) {
      if (g_slice_has_steps[a] && g_slice_steps_left[a] > 0) {
        queued = false;
      }
      if (!g_slice_had_steps[a]) {
        any_idle = true;
      }
    }
    if (queued) {
      if (any_idle) {
        g_slice_drain_wait = true;
      } else {
        g_slice_in_progress = false;
      }
    }
    if (!progressed && g_slice_in_progress) {
      break;
    }
  }
  return emitted;
}

#else /* HOST_TEST */

bool motion_path_go(void) {
  if (g_path_count == 0) {
    return false;
  }
  return motion_path_go_range(0, g_path_count - 1);
}

bool motion_path_go_range(uint32_t start_index, uint32_t end_index) {
  if (!path_go_common(start_index, end_index)) {
    return false;
  }
  motion_end_joy();
  g_path_active = true;
  return true;
}

void motion_path_abort_to_planner(void) {
  g_path_active = false;
  g_slice_in_progress = false;
  g_slice_drain_wait = false;
  g_path_range_done = false;
}

int motion_path_fill_fifo(void) { return 0; }

#endif /* HOST_TEST */
