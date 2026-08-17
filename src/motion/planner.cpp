#include "planner.h"
#include "planner_math.h"
#include "pio_step.h"
#include "config_store.h"
#include "motion_diag.h"
#include "pins.h"
#include "debug_hw.h"
#include "protocol.h"
#include "protocol_internal.h"
#include "config_defaults.h"

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#ifndef HOST_TEST
#include <Arduino.h>
#endif

#define HARD_LIMIT_DEBOUNCE_S 0.020f

typedef enum {
  HOME_IDLE = 0,
  HOME_CLEAR,
  HOME_SEEK,
  HOME_BACKOFF_REL,
  HOME_BACKOFF_OUT
} HomePhase;

/*
 * Sine seek planner — one rate truth at FIFO fill time.
 * Position advances when steps are committed to TX (queued).
 * pack_n never emits past remaining_steps (pendeln guard).
 */

static McStatus g_st;
static float g_cruise_mm_s;
static float g_accel_mm_s2;

static int64_t g_pos_steps;
static int64_t g_target_steps;
static float g_vel_mm_s; /* signed issue velocity */
static float g_ramp_v0;
static float g_ramp_v1;
static float g_ramp_phi;
static bool g_ramp_active;

static bool g_stopping;
static bool g_dir_pause;
static float g_dir_pause_s;
static int g_last_sign;
/* Last fill still had steps to issue — only then is a dry FIFO a real underrun. */
static bool g_fill_wants_more;
/* Measured dv/dt of issued words: covers both ramp and stop-distance braking. */
static float g_acc_meas;
/* Committed to the braking ramp: keeps the S-curve instead of re-planning it. */
static bool g_braking;
/* Committed brake arc, keyed on distance so it cannot drift as v -> 0:
 * total steps, position at commit, entry speed. */
static int g_brake_d;
static int64_t g_brake_pos0;
static float g_brake_v0;

static float g_spmm;

/* Hard limits: latches + debounced stable levels */
static bool g_hl_l_latched;
static bool g_hl_r_latched;
static bool g_hl_l_stable; /* debounced asserted */
static bool g_hl_r_stable;
static float g_hl_l_timer_s;
static float g_hl_r_timer_s;
static bool g_hl_l_raw_prev;
static bool g_hl_r_raw_prev;

/* Home switch: debounced level only (for MH; never hard-limit trip) */
static bool g_home_stable;
static float g_home_timer_s;
static bool g_home_raw_prev;

/* DRV_ERROR / E-stop input */
static bool g_drv_err_stable;
static float g_drv_err_timer_s;
static bool g_drv_err_raw_prev;
static bool g_drv_err_seeded;

static HomePhase g_home_phase;
static int g_home_sign; /* seek direction: -1 left, +1 right */
static int64_t g_home_seek_start_pos;
static int64_t g_home_max_travel_steps;
static float g_home_saved_cruise;
static float g_home_saved_accel;
static bool g_home_speeds_saved;

static void reset_ramp(void);
static void begin_ramp(float v_cmd);
static void home_restore_speeds(void);
static void home_begin_seek(void);
static void home_begin_backoff(void);
static void home_begin_clear(int clear_sign);
static void home_finish(void);
static void home_abort(const char *msg);
static void home_start_move_sign(int sign, float dist_mm);
static bool home_ref_asserted(void);
static int home_seek_sign(void);
static bool home_cfg_ok(void);
static void home_poll_fsm(void);

static bool read_limit_raw(bool left) {
#ifndef HOST_TEST
  const McConfig *c = config_get();
  if (left) {
    if (!c->sw_limit_l_use) {
      return false;
    }
    int level = digitalRead(PIN_SW_LIMIT_L) ? 1 : 0;
    return config_pin_asserted(level, c->sw_limit_l_active);
  }
  if (!c->sw_limit_r_use) {
    return false;
  }
  int level = digitalRead(PIN_SW_LIMIT_R) ? 1 : 0;
  return config_pin_asserted(level, c->sw_limit_r_active);
#else
  (void)left;
  return false;
#endif
}

static bool read_home_raw(void) {
#ifndef HOST_TEST
  const McConfig *c = config_get();
  if (!c->sw_home_use) {
    return false;
  }
  int level = digitalRead(PIN_SW_HOME) ? 1 : 0;
  return config_pin_asserted(level, c->sw_home_active);
#else
  return false;
#endif
}

static bool read_drv_error_raw(void) {
#ifndef HOST_TEST
  const McConfig *c = config_get();
  int level = digitalRead(PIN_DRV_ERROR) ? 1 : 0;
  return config_pin_asserted(level, c->drv_error_active);
#else
  return false;
#endif
}

static void apply_en_output(bool enabled) {
#ifndef HOST_TEST
  const McConfig *c = config_get();
  if (enabled) {
    digitalWrite(PIN_DRV_EN, c->drv_en_active ? HIGH : LOW);
  } else {
    digitalWrite(PIN_DRV_EN, c->drv_en_active ? LOW : HIGH);
  }
#else
  (void)enabled;
#endif
}

static void refresh_state(void) {
  g_st.hard_limit = g_hl_l_latched || g_hl_r_latched || g_hl_l_stable || g_hl_r_stable;
  if (g_st.drv_error) {
    g_st.state = MC_STATE_ERROR;
  } else if (g_st.hard_limit && !g_st.moving) {
    g_st.state = MC_STATE_HARD_LIMIT;
  } else if (!g_st.enabled) {
    g_st.state = MC_STATE_DISABLED;
  } else if (g_st.homing) {
    g_st.state = MC_STATE_HOMING;
  } else if (g_st.moving) {
    if (g_braking || g_stopping) {
      g_st.state = MC_STATE_DECELERATING;
    } else if (g_ramp_active) {
      /* Mid-move SS change and start/stop ramps share begin_ramp(v0→v1). */
      if (fabsf(g_ramp_v1) + 1e-4f < fabsf(g_ramp_v0)) {
        g_st.state = MC_STATE_DECELERATING;
      } else if (fabsf(g_ramp_v1) > fabsf(g_ramp_v0) + 1e-4f) {
        g_st.state = MC_STATE_ACCELERATING;
      } else {
        g_st.state = MC_STATE_MOVING;
      }
    } else {
      g_st.state = MC_STATE_MOVING;
    }
  } else {
    g_st.state = MC_STATE_IDLE;
  }
  g_st.pos_mm = (float)g_pos_steps / g_spmm;
  g_st.target_mm = (float)g_target_steps / g_spmm;
  g_st.vel_mm_s = g_vel_mm_s;
  /* Measured |a| of the issued words; 0 in cruise/idle. */
  g_st.acc_mm_s2 = g_st.moving ? fabsf(g_acc_meas) : 0.0f;
  g_st.has_target = g_st.moving && !g_stopping;
  dbg_hw_set(PIN_DBG_MOV, g_st.moving || g_st.homing);
}

/**
 * Emergency halt: immediate STEP abort, EN off, cancel waits/chain.
 * Used by HT/H, hard-limit trips, and PIN_DRV_ERROR.
 */
static void planner_halt(void) {
  protocol_debug(3, "D:halt\n");
  g_vel_mm_s = 0.0f;
  g_st.moving = false;
  g_st.homing = false;
  g_home_phase = HOME_IDLE;
  home_restore_speeds();
  g_stopping = false;
  g_dir_pause = false;
  g_st.has_target = false;
  g_target_steps = g_pos_steps;
  reset_ramp();
  pio_step_stop_hard();

  g_st.enabled = false;
  apply_en_output(false);
  protocol_cancel_waits_and_chain();
  refresh_state();
}

static void planner_trip_hard_limit(bool left) {
  if (g_st.homing) {
    int hm = config_get()->home_mode;
    bool is_ref = (hm == 3 && left) || (hm == 4 && !left);
    if (is_ref && g_home_phase == HOME_SEEK) {
      /* Reference limit hit — enter backoff, do not fault. */
      home_begin_backoff();
      return;
    }
    if (is_ref && (g_home_phase == HOME_BACKOFF_REL || g_home_phase == HOME_BACKOFF_OUT)) {
      return;
    }
    if (hm == 1 || hm == 2) {
      protocol_error("home", "hard");
      /* fall through to hard-stop fault */
    }
  }

  if (left) {
    if (g_hl_l_latched) {
      return;
    }
    g_hl_l_latched = true;
  } else {
    if (g_hl_r_latched) {
      return;
    }
    g_hl_r_latched = true;
  }
  planner_halt();
}

bool planner_hard_limit_blocks_sign(int sign) {
  if (sign < 0 && (g_hl_l_latched || g_hl_l_stable)) {
    return true;
  }
  if (sign > 0 && (g_hl_r_latched || g_hl_r_stable)) {
    return true;
  }
  return false;
}

bool planner_home_switch_asserted(void) {
  return g_home_stable;
}

bool planner_home_switch_enabled(void) {
  return config_get()->sw_home_use != 0;
}

static void debounce_side(bool left, float dt_s) {
  const McConfig *c = config_get();
  bool use = left ? c->sw_limit_l_use : c->sw_limit_r_use;
  bool *stable = left ? &g_hl_l_stable : &g_hl_r_stable;
  bool *latched = left ? &g_hl_l_latched : &g_hl_r_latched;
  float *timer = left ? &g_hl_l_timer_s : &g_hl_r_timer_s;
  bool *raw_prev = left ? &g_hl_l_raw_prev : &g_hl_r_raw_prev;

  if (!use) {
    *stable = false;
    *latched = false;
    *timer = 0.0f;
    *raw_prev = false;
    return;
  }

#ifndef HOST_TEST
  /* Lazy init if CS enables use after boot. */
  pinMode(left ? PIN_SW_LIMIT_L : PIN_SW_LIMIT_R, INPUT_PULLUP);
#endif

  bool raw = read_limit_raw(left);
  if (raw != *raw_prev) {
    *timer = 0.0f;
    *raw_prev = raw;
  } else {
    *timer += dt_s;
  }

  if (*timer >= HARD_LIMIT_DEBOUNCE_S) {
    if (raw && !*stable) {
      *stable = true;
      planner_trip_hard_limit(left);
    } else if (!raw && *stable) {
      *stable = false;
      *latched = false;
    }
  }
}

static void debounce_home(float dt_s) {
  const McConfig *c = config_get();
  if (!c->sw_home_use) {
    g_home_stable = false;
    g_home_timer_s = 0.0f;
    g_home_raw_prev = false;
    return;
  }

#ifndef HOST_TEST
  pinMode(PIN_SW_HOME, INPUT_PULLUP);
#endif

  bool raw = read_home_raw();
  if (raw != g_home_raw_prev) {
    g_home_timer_s = 0.0f;
    g_home_raw_prev = raw;
  } else {
    g_home_timer_s += dt_s;
  }

  if (g_home_timer_s >= HARD_LIMIT_DEBOUNCE_S) {
    g_home_stable = raw;
  }
}

static void debounce_drv_error(float dt_s) {
#ifndef HOST_TEST
  pinMode(PIN_DRV_ERROR, INPUT_PULLUP);
#endif

  bool raw = read_drv_error_raw();
  if (!g_drv_err_seeded) {
    /* Power-up: seed from first sample so an already-asserted pin can become stable. */
    g_drv_err_raw_prev = raw;
    g_drv_err_timer_s = 0.0f;
    g_drv_err_seeded = true;
  } else if (raw != g_drv_err_raw_prev) {
    g_drv_err_timer_s = 0.0f;
    g_drv_err_raw_prev = raw;
  } else {
    g_drv_err_timer_s += dt_s;
  }

  if (g_drv_err_timer_s >= HARD_LIMIT_DEBOUNCE_S) {
    if (raw && !g_drv_err_stable) {
      g_drv_err_stable = true;
      g_st.drv_error = true;
      planner_halt();
    } else if (!raw && g_drv_err_stable) {
      g_drv_err_stable = false;
      g_st.drv_error = false;
      refresh_state();
    }
  }
}

static void planner_poll_switches(float dt_s) {
  debounce_side(true, dt_s);
  debounce_side(false, dt_s);
  debounce_home(dt_s);
  debounce_drv_error(dt_s);
}

static int64_t mm_to_steps(float mm) {
  return (int64_t)lroundf(mm * g_spmm);
}

static void home_restore_speeds(void) {
  if (!g_home_speeds_saved) {
    return;
  }
  g_cruise_mm_s = g_home_saved_cruise;
  g_accel_mm_s2 = g_home_saved_accel;
  g_home_speeds_saved = false;
}

static int home_seek_sign(void) {
  int hm = config_get()->home_mode;
  return (hm == 1 || hm == 3) ? -1 : 1;
}

static bool home_ref_asserted(void) {
  int hm = config_get()->home_mode;
  if (hm == 1 || hm == 2) {
    return g_home_stable;
  }
  if (hm == 3) {
    return g_hl_l_stable;
  }
  if (hm == 4) {
    return g_hl_r_stable;
  }
  return false;
}

static bool home_cfg_ok(void) {
  const McConfig *c = config_get();
  switch (c->home_mode) {
  case 1:
  case 2:
    return c->sw_home_use != 0;
  case 3:
    return c->sw_limit_l_use != 0;
  case 4:
    return c->sw_limit_r_use != 0;
  default:
    return false;
  }
}

static void home_start_move_sign(int sign, float dist_mm) {
  if (sign == 0 || dist_mm < 0.0f) {
    return;
  }
  g_spmm = config_get()->steps_per_mm;
  if (g_spmm < 1e-3f) {
    g_spmm = 1.0f;
  }
  int64_t delta = mm_to_steps(dist_mm);
  if (delta < 1) {
    delta = 1;
  }
  g_target_steps = g_pos_steps + (int64_t)sign * delta;
  g_stopping = false;
  g_st.moving = true;
  g_st.has_target = true;
  begin_ramp(g_vel_mm_s);
  g_fill_wants_more = false;
  g_acc_meas = 0.0f;
  g_braking = false;
  g_brake_d = 0;
  pio_step_clear_stall(); /* stale flag from the previous idle gap */
  pio_step_start();
  pio_step_kick_feed();
}

static void home_begin_seek(void) {
  const McConfig *c = config_get();
  g_home_phase = HOME_SEEK;
  g_home_sign = home_seek_sign();
  g_home_seek_start_pos = g_pos_steps;
  float span_mm;
  if (!isnan(c->slider_min_mm) && !isnan(c->slider_max_mm) && c->slider_max_mm > c->slider_min_mm) {
    span_mm = c->slider_max_mm - c->slider_min_mm;
  } else {
    span_mm = CFG_DEFAULT_SLIDER_MAX_MM - CFG_DEFAULT_SLIDER_MIN_MM;
  }
  float max_mm = span_mm * 1.1f;
  if (max_mm < 1.0f) {
    max_mm = 1.0f;
  }
  g_home_max_travel_steps = mm_to_steps(max_mm);
  if (g_home_max_travel_steps < 1) {
    g_home_max_travel_steps = 1;
  }
  home_start_move_sign(g_home_sign, max_mm);
}

static void home_begin_backoff(void) {
  g_home_phase = HOME_BACKOFF_REL;
  /* Leave reference switch; clear latches on reference limit so drive-out works. */
  int hm = config_get()->home_mode;
  if (hm == 3) {
    g_hl_l_latched = false;
  } else if (hm == 4) {
    g_hl_r_latched = false;
  }
  float span = 50.0f;
  const McConfig *c = config_get();
  if (!isnan(c->slider_min_mm) && !isnan(c->slider_max_mm) && c->slider_max_mm > c->slider_min_mm) {
    span = (c->slider_max_mm - c->slider_min_mm) * 0.25f;
  }
  if (span < 10.0f) {
    span = 10.0f;
  }
  home_start_move_sign(-g_home_sign, span);
}

static void home_begin_clear(int clear_sign) {
  g_home_phase = HOME_CLEAR;
  if (clear_sign < 0) {
    g_hl_l_latched = false;
  } else if (clear_sign > 0) {
    g_hl_r_latched = false;
  }
  float span = 50.0f;
  const McConfig *c = config_get();
  if (!isnan(c->slider_min_mm) && !isnan(c->slider_max_mm) && c->slider_max_mm > c->slider_min_mm) {
    span = (c->slider_max_mm - c->slider_min_mm) * 0.25f;
  }
  if (span < 10.0f) {
    span = 10.0f;
  }
  home_start_move_sign(clear_sign, span);
}

static void home_finish(void) {
  const McConfig *c = config_get();
  float pos_mm;
  if (c->home_mode == 1 || c->home_mode == 3) {
    pos_mm = isnan(c->slider_min_mm) ? 0.0f : c->slider_min_mm;
  } else {
    pos_mm = isnan(c->slider_max_mm) ? 0.0f : c->slider_max_mm;
  }
  g_spmm = c->steps_per_mm;
  if (g_spmm < 1e-3f) {
    g_spmm = 1.0f;
  }
  g_pos_steps = mm_to_steps(pos_mm);
  g_target_steps = g_pos_steps;
  g_vel_mm_s = 0.0f;
  g_st.moving = false;
  g_st.homing = false;
  g_st.has_target = false;
  g_stopping = false;
  g_home_phase = HOME_IDLE;
  reset_ramp();
  home_restore_speeds();
  pio_step_stop_soft();
  refresh_state();
}

static void home_abort(const char *msg) {
  g_home_phase = HOME_IDLE;
  g_st.homing = false;
  g_st.moving = false;
  g_st.has_target = false;
  g_stopping = false;
  g_dir_pause = false;
  g_vel_mm_s = 0.0f;
  g_target_steps = g_pos_steps;
  reset_ramp();
  home_restore_speeds();
  pio_step_stop_hard();
  protocol_error("home", msg ? msg : "abort");
  protocol_cancel_waits_and_chain();
  refresh_state();
}

static void home_poll_fsm(void) {
  if (!g_st.homing || g_home_phase == HOME_IDLE || g_stopping) {
    return;
  }

  if (g_home_phase == HOME_CLEAR) {
    if (!g_hl_l_stable && !g_hl_r_stable) {
      g_hl_l_latched = false;
      g_hl_r_latched = false;
      g_vel_mm_s = 0.0f;
      g_target_steps = g_pos_steps;
      reset_ramp();
      pio_step_stop_hard();
      if (home_ref_asserted()) {
        home_begin_backoff();
      } else {
        home_begin_seek();
      }
    }
    return;
  }

  if (g_home_phase == HOME_SEEK) {
    int64_t traveled = g_pos_steps - g_home_seek_start_pos;
    if (traveled < 0) {
      traveled = -traveled;
    }
    if (traveled >= g_home_max_travel_steps) {
      home_abort("travel");
      return;
    }
    if (home_ref_asserted()) {
      home_begin_backoff();
    }
    return;
  }

  if (g_home_phase == HOME_BACKOFF_REL) {
    if (!home_ref_asserted()) {
      float out = config_get()->home_move_out_mm;
      if (out < 0.0f) {
        out = 0.0f;
      }
      g_home_phase = HOME_BACKOFF_OUT;
      if (out < 1e-4f) {
        home_finish();
      } else {
        home_start_move_sign(-g_home_sign, out);
      }
    }
    return;
  }

  if (g_home_phase == HOME_BACKOFF_OUT) {
    if (g_pos_steps == g_target_steps && fabsf(g_vel_mm_s) < 0.01f && pio_step_tx_empty()) {
      home_finish();
    }
  }
}

static void reset_ramp(void) {
  g_ramp_active = false;
  g_ramp_v0 = g_vel_mm_s;
  g_ramp_v1 = g_vel_mm_s;
  g_ramp_phi = 0.0f;
}

static void begin_ramp(float v_cmd) {
  if (!g_ramp_active || fabsf(v_cmd - g_ramp_v1) > 1e-4f) {
    g_ramp_v0 = g_vel_mm_s;
    g_ramp_v1 = v_cmd;
    g_ramp_phi = 0.0f;
    g_ramp_active = true;
  }
}

static int64_t soft_min_steps(void) {
  const McConfig *c = config_get();
  if (isnan(c->slider_min_mm)) {
    return INT64_MIN / 4;
  }
  return mm_to_steps(c->slider_min_mm);
}

static int64_t soft_max_steps(void) {
  const McConfig *c = config_get();
  if (isnan(c->slider_max_mm)) {
    return INT64_MAX / 4;
  }
  return mm_to_steps(c->slider_max_mm);
}

static int remaining_steps_for_sign(int sign) {
  int64_t rem_tgt = g_target_steps - g_pos_steps;
  if (sign > 0 && rem_tgt < 0) {
    rem_tgt = 0;
  }
  if (sign < 0 && rem_tgt > 0) {
    rem_tgt = 0;
  }
  int rem = (int)(rem_tgt >= 0 ? rem_tgt : -rem_tgt);

  /* Soft limits do not constrain homing moves. */
  if (!g_st.homing) {
    if (sign > 0) {
      int64_t to_lim = soft_max_steps() - g_pos_steps;
      if (to_lim < 0) {
        to_lim = 0;
      }
      if (to_lim < rem) {
        rem = (int)to_lim;
      }
    } else if (sign < 0) {
      int64_t to_lim = g_pos_steps - soft_min_steps();
      if (to_lim < 0) {
        to_lim = 0;
      }
      if (to_lim < rem) {
        rem = (int)to_lim;
      }
    }
  }
  return rem;
}

static float cruise_cap(void) {
  float v = g_cruise_mm_s;
  float mx = config_get()->max_speed_mm_s;
  if (v > mx) {
    v = mx;
  }
  float hz_max = planner_max_step_hz(pio_step_sysclk_hz(), PIO_STEP_PERIOD_FIXED);
  float v_hz = hz_max / g_spmm;
  if (v > v_hz) {
    v = v_hz;
  }
  return v;
}

static void settle_if_done(void) {
  int64_t err = g_target_steps - g_pos_steps;
  if (g_stopping) {
    if (fabsf(g_vel_mm_s) < 0.01f && pio_step_tx_empty()) {
      g_vel_mm_s = 0.0f;
      g_st.moving = false;
      g_st.homing = false;
      g_home_phase = HOME_IDLE;
      home_restore_speeds();
      g_stopping = false;
      g_target_steps = g_pos_steps;
      reset_ramp();
      pio_step_stop_soft();
      refresh_state();
    }
    return;
  }
  if (g_st.homing && g_home_phase != HOME_IDLE) {
    /* Homing FSM owns completion (home_poll_fsm / home_finish). */
    return;
  }
  if (err == 0 && fabsf(g_vel_mm_s) < 0.01f) {
    g_vel_mm_s = 0.0f;
    g_st.moving = false;
    g_st.homing = false;
    reset_ramp();
    if (pio_step_tx_empty()) {
      pio_step_stop_soft();
    }
    refresh_state();
  }
}

void planner_init(void) {
  memset(&g_st, 0, sizeof(g_st));
  g_st.state = MC_STATE_DISABLED;
  g_spmm = config_get()->steps_per_mm;
  if (g_spmm < 1e-3f) {
    g_spmm = 1.0f;
  }
  g_cruise_mm_s = session_get()->speed_mm_s;
  g_accel_mm_s2 = session_get()->accel_mm_s2;
  g_pos_steps = 0;
  g_target_steps = 0;
  g_vel_mm_s = 0.0f;
  g_stopping = false;
  g_dir_pause = false;
  g_dir_pause_s = 0.0f;
  g_last_sign = 0;
  g_fill_wants_more = false;
  g_acc_meas = 0.0f;
  g_braking = false;
  g_brake_d = 0;
  g_hl_l_latched = false;
  g_hl_r_latched = false;
  g_hl_l_stable = false;
  g_hl_r_stable = false;
  g_hl_l_timer_s = 0.0f;
  g_hl_r_timer_s = 0.0f;
  g_hl_l_raw_prev = false;
  g_hl_r_raw_prev = false;
  g_home_stable = false;
  g_home_timer_s = 0.0f;
  g_home_raw_prev = false;
  g_drv_err_stable = false;
  g_drv_err_timer_s = 0.0f;
  g_drv_err_raw_prev = false;
  g_drv_err_seeded = false;
  g_home_phase = HOME_IDLE;
  g_home_sign = 0;
  g_home_seek_start_pos = 0;
  g_home_max_travel_steps = 0;
  g_home_speeds_saved = false;
  reset_ramp();
  motion_diag_init_from_noinit();
  pio_step_init();
  refresh_state();
}

int planner_fill_fifo(void) {
  int emitted = 0;
  if (!g_st.moving || g_st.drv_error || !g_st.enabled) {
    g_fill_wants_more = false;
    return 0;
  }
  if (g_dir_pause) {
    g_fill_wants_more = false;
    return 0;
  }

  unsigned room_budget = 4;
  while (room_budget-- && pio_step_tx_room() > 0) {
    int64_t err = g_target_steps - g_pos_steps;
    int sign;
    if (g_stopping) {
      if (fabsf(g_vel_mm_s) < 0.01f) {
        g_fill_wants_more = false;
        break;
      }
      sign = (g_vel_mm_s >= 0.0f) ? 1 : -1;
    } else if (err > 0) {
      sign = 1;
    } else if (err < 0) {
      sign = -1;
    } else {
      /* At target: finish deceleration, but stop rather than creep past it for
         a residue that is only worth a few steps. The approach floor leaves the
         axis at stop_approach_hz, so that speed also counts as arrived —
         otherwise the trailing decel would issue steps beyond the target. */
      int stop_app_hz = config_get()->stop_approach_hz;
      float v_stop = (stop_app_hz > 0) ? (float)stop_app_hz / g_spmm : 0.0f;
      if (fabsf(g_vel_mm_s) <= v_stop + 1e-3f ||
          planner_stop_rem_steps(g_vel_mm_s, g_accel_mm_s2, g_spmm) <= 4) {
        g_vel_mm_s = 0.0f;
        g_fill_wants_more = false;
        break;
      }
      sign = (g_vel_mm_s >= 0.0f) ? 1 : -1;
    }

    /* Reverse: decelerate in current travel direction until ~0, then pause. */
    bool reverse_decel = false;
    if (!g_stopping && planner_needs_reverse_decel(g_last_sign, sign, g_vel_mm_s)) {
      begin_ramp(0.0f);
      sign = (g_vel_mm_s >= 0.0f) ? 1 : -1;
      reverse_decel = true;
      protocol_debug(4, "D:reverse_decel v=%.2f\n", (double)g_vel_mm_s);
    }

    int rem;
    if (g_stopping || err == 0 || reverse_decel) {
      rem = planner_stop_rem_steps(g_vel_mm_s, g_accel_mm_s2, g_spmm);
    } else {
      rem = remaining_steps_for_sign(sign);
    }

    if (rem <= 0) {
      if (err != 0 && !g_stopping && !reverse_decel) {
        /* Nothing left to issue although the target is not reached (soft limit).
           Drop the target so settle_if_done() can leave the moving state. */
        motion_diag_note_overshoot(1);
        protocol_debug(2, "D:move_blocked pos=%ld tgt=%ld\n", (long)g_pos_steps,
                       (long)g_target_steps);
        g_target_steps = g_pos_steps;
      }
      g_vel_mm_s = 0.0f;
      reset_ramp();
      g_braking = false;
      g_brake_d = 0;
      g_fill_wants_more = false;
      break;
    }

    float rem_mm = (float)rem / g_spmm;
    float vmax = planner_vmax_for_distance(rem_mm, g_accel_mm_s2);
    /*
     * Ramp target is either cruise or 0. Feeding it min(cruise, vmax) would
     * move the target every word as rem shrinks, restart the sine phase, and
     * leave the axis stuck near ramp_start_hz (seen as "higher ss → slower").
     * When remaining distance cannot support the current speed, brake to 0;
     * the vmax clamp below is the hard safety net for the issued word.
     */
    float v_cmd;
    if (g_stopping || reverse_decel) {
      v_cmd = 0.0f;
    } else if (g_braking || fabsf(g_vel_mm_s) > vmax) {
      /* Braking point reached: commit to one ramp down to 0. Re-deriving it per
         word would restart the sine and flatten it into constant 2a/pi. */
      g_braking = true;
      v_cmd = 0.0f;
    } else {
      v_cmd = (float)sign * cruise_cap();
    }

    bool decel = g_stopping || reverse_decel || g_braking;
    if (decel) {
      /* Commit the brake arc once and then track progress along it by distance.
         A soft-stop rem is re-derived from v every word and would drift; keyed
         on distance the profile lands on the target by construction. */
      if (g_brake_d <= 0) {
        g_brake_d = rem;
        g_brake_pos0 = g_pos_steps;
        g_brake_v0 = fabsf(g_vel_mm_s);
      }
      rem = g_brake_d - (int)llabs(g_pos_steps - g_brake_pos0);
      if (rem <= 0 || g_brake_v0 <= 0.0f) {
        g_vel_mm_s = 0.0f;
        reset_ramp();
        g_braking = false;
        g_brake_d = 0;
        g_fill_wants_more = false;
        break;
      }
    } else {
      g_brake_d = 0;
    }

    /* Direction change at zero crossing */
    if (!g_stopping && g_last_sign != 0 && sign != g_last_sign && fabsf(g_vel_mm_s) < 0.05f) {
      float pause = config_get()->dir_change_pause_s;
      if (pause > 0.0f) {
        g_dir_pause = true;
        g_dir_pause_s = pause;
        g_vel_mm_s = 0.0f;
        reset_ramp();
        g_braking = false;
        g_brake_d = 0;
        g_fill_wants_more = false;
        protocol_debug(4, "D:dir_pause %.3f\n", (double)pause);
        break;
      }
    }

    int ramp_start = config_get()->ramp_start_hz;
    int stop_app = config_get()->stop_approach_hz;
    /*
     * Leave standstill at ramp_start_hz. A sine from v=0 has near-zero initial
     * accel, and the old floor kept STEP stuck at ramp_start while phi crawled
     * up — felt as a long constant low-speed creep (worse with small SA).
     * Seed vmin so the S-curve accelerates immediately toward cruise.
     */
    if (ramp_start > 0 && !g_stopping && !reverse_decel && !g_braking) {
      float vmin = (float)ramp_start / g_spmm;
      if (vmin > 0.0f && fabsf(g_vel_mm_s) < vmin && fabsf(v_cmd) > vmin) {
        g_vel_mm_s = (float)sign * vmin;
        reset_ramp();
      }
    }

    begin_ramp(v_cmd);

    /*
     * ramp_start_hz is the *launch* floor only. Applying it while braking pinned
     * STEP at ramp_start for the last d_stop(ramp_start) — the axis then halted
     * from that rate instead of tapering (end-of-move snap). Approaching the
     * target is bounded by stop_approach_hz instead.
     */
    float min_hz = (decel && stop_app > 0) ? (float)stop_app : (float)ramp_start;

    float step_hz_est = fabsf(g_vel_mm_s) * g_spmm;
    if (step_hz_est < 1.0f) {
      step_hz_est = min_hz;
    }
    if (!decel && rem > 1 && step_hz_est < (float)ramp_start) {
      step_hz_est = (float)ramp_start;
    }
    /* Approach floor: avoid crawling below stop_approach_hz while braking in. */
    if ((decel || rem <= 4) && stop_app > 0 && step_hz_est > 0.0f &&
        step_hz_est < (float)stop_app) {
      step_hz_est = (float)stop_app;
    }

    int pending = pio_step_pending_steps();
    int n = planner_pack_n(step_hz_est, rem, pending);
    if (n <= 0) {
      /* Enough steps already queued for the time budget. */
      g_fill_wants_more = false;
      break;
    }
    if (n > rem) {
      n = rem;
    }

    /* Advance ramp by the real duration of the packed word (n pulses). */
    float dt = (float)n / step_hz_est;
    float v_prev = g_vel_mm_s;
    if (decel) {
      /*
       * Brake speed as a closed form of the distance still to run:
       *   v(R) = v_app + (v0 - v_app) * sin(pi*R / (2*D))
       * Integrating the ramp in time instead let it lag (dphi/ds = 1/(v*T) is
       * stiff as v -> 0); the v_cap clamp then took over and stopped the axis at
       * full accel — the end-of-move snap. Keyed on distance it cannot drift.
       * Landing on v_app (stop_approach) instead of 0 keeps the arc finite: a
       * tail decaying to zero takes exponentially long and crawls on the floor.
       */
      float v_app = (stop_app > 0) ? (float)stop_app / g_spmm : 0.0f;
      float dv = g_brake_v0 - v_app;
      if (dv < 0.0f) {
        dv = 0.0f;
      }
      int r_after = rem - n;
      if (r_after <= 0) {
        g_vel_mm_s = (float)sign * v_app;
      } else {
        float frac = (float)M_PI * (float)r_after / (2.0f * (float)g_brake_d);
        g_vel_mm_s = (float)sign * (v_app + dv * sinf(frac));
      }
    } else {
      g_ramp_phi = planner_sine_advance_phi(g_ramp_phi, g_ramp_v0, g_ramp_v1, g_accel_mm_s2, dt);
      g_vel_mm_s = planner_sine_vel(g_ramp_v0, g_ramp_v1, g_ramp_phi);
      if (g_ramp_phi >= 1.0f) {
        g_vel_mm_s = g_ramp_v1;
        g_ramp_active = false;
      }
    }

    /* Clamp to stop distance. While braking this is only an outer safety cap,
       so the committed ramp is not torn down by it. */
    float v_cap = g_braking ? vmax * 1.25f : vmax;
    if (fabsf(g_vel_mm_s) > v_cap) {
      g_vel_mm_s = (g_vel_mm_s >= 0.0f) ? v_cap : -v_cap;
      if (!g_braking) {
        reset_ramp();
      }
    }

    float step_hz = fabsf(g_vel_mm_s) * g_spmm;
    if (!decel && rem > 1 && step_hz < (float)ramp_start) {
      step_hz = (float)ramp_start;
    }
    if ((decel || rem <= 4) && stop_app > 0 && step_hz > 0.0f &&
        step_hz < (float)stop_app) {
      step_hz = (float)stop_app;
    }
    if (step_hz < 1.0f) {
      if (g_stopping || reverse_decel || err == 0) {
        g_vel_mm_s = 0.0f;
        g_fill_wants_more = false;
        break;
      }
      step_hz = 1.0f;
    }

    uint32_t delay =
        planner_hz_to_delay(step_hz, pio_step_sysclk_hz(), PIO_STEP_PERIOD_FIXED);
    pio_step_set_dir(sign);
    if (!pio_step_put_word(delay, (uint8_t)n)) {
      /* TX full: work remains, so a dry FIFO later would be a real underrun. */
      g_fill_wants_more = true;
      break;
    }
    g_fill_wants_more = true;
    motion_diag_note_hz(step_hz);
    if (dt > 1e-6f) {
      float a_now = (g_vel_mm_s - v_prev) / dt;
      g_acc_meas += 0.25f * (a_now - g_acc_meas);
    }
    int64_t pos_before = g_pos_steps;
    g_pos_steps += (int64_t)n * (int64_t)sign;
    g_last_sign = sign;
    ++emitted;

    /*
     * Overshoot = this word carried the axis across the target. Being *beyond*
     * the target is normal while the trailing deceleration is issued (err == 0)
     * or while a reverse move bleeds off speed, so only a crossing counts.
     */
    if (!g_stopping && !reverse_decel && err != 0) {
      bool crossed = (sign > 0 && pos_before <= g_target_steps && g_pos_steps > g_target_steps) ||
                     (sign < 0 && pos_before >= g_target_steps && g_pos_steps < g_target_steps);
      if (crossed) {
        motion_diag_note_overshoot((int)llabs(g_pos_steps - g_target_steps));
        protocol_debug(2, "D:overshoot %ld\n", (long)llabs(g_pos_steps - g_target_steps));
        g_pos_steps = g_target_steps;
        g_vel_mm_s = 0.0f;
        reset_ramp();
        g_braking = false;
        g_brake_d = 0;
        g_fill_wants_more = false;
        break;
      }
    }
  }
  refresh_state();
  return emitted;
}

void planner_tick(float dt_s) {
  if (dt_s < 0.0f) {
    dt_s = 0.0f;
  }
  planner_poll_switches(dt_s);
  home_poll_fsm();
  if (g_dir_pause) {
    g_dir_pause_s -= dt_s;
    if (g_dir_pause_s <= 0.0f) {
      g_dir_pause = false;
      g_dir_pause_s = 0.0f;
      g_last_sign = 0;
      pio_step_clear_stall(); /* idle stall during the pause is not an underrun */
    }
  }

  if (g_st.moving && !g_dir_pause && g_fill_wants_more && pio_step_is_stalled() &&
      pio_step_tx_empty()) {
    motion_diag_note_underrun();
    protocol_debug(2, "D:underrun\n");
#ifndef HOST_TEST
#ifdef DEBUG_HW
    dbg_hw_set(PIN_DBG_UNDERRUN, 1);
    dbg_hw_set(PIN_DBG_UNDERRUN, 0);
#endif
#endif
  }

  if (g_st.moving) {
    /* FIFO fill is owned solely by task_motion_feed (avoids dual-caller race). */
    if (!pio_step_tx_empty() || fabsf(g_vel_mm_s) > 0.01f ||
        g_pos_steps != g_target_steps || g_stopping) {
      pio_step_start();
    }
    settle_if_done();
  }
  home_poll_fsm();
  refresh_state();
}

static bool clamp_target_mm(float *mm) {
  const McConfig *c = config_get();
  bool ok = true;
  if (!isnan(c->slider_min_mm) && *mm < c->slider_min_mm) {
    *mm = c->slider_min_mm;
    g_st.at_soft_limit = true;
    ok = false;
  } else if (!isnan(c->slider_max_mm) && *mm > c->slider_max_mm) {
    *mm = c->slider_max_mm;
    g_st.at_soft_limit = true;
    ok = false;
  } else {
    g_st.at_soft_limit = false;
  }
  return ok;
}

void planner_request_move_to(float mm) {
  if (!g_st.enabled || g_st.drv_error) {
    return;
  }
  clamp_target_mm(&mm);
  g_spmm = config_get()->steps_per_mm;
  float pos_mm = (float)g_pos_steps / g_spmm;
  int sign = 0;
  if (mm > pos_mm + 1e-6f) {
    sign = 1;
  } else if (mm < pos_mm - 1e-6f) {
    sign = -1;
  }
  if (planner_hard_limit_blocks_sign(sign)) {
    return;
  }
  g_target_steps = mm_to_steps(mm);
  g_stopping = false;
  g_st.moving = true;
  g_st.homing = false;
  g_st.has_target = true;
  begin_ramp(g_vel_mm_s); /* retarget keeps velocity; new ramp to new cmd on fill */
  g_fill_wants_more = false;
  g_acc_meas = 0.0f;
  g_braking = false;
  g_brake_d = 0;
  pio_step_clear_stall(); /* stale flag from the previous idle gap */
  pio_step_start();
  pio_step_kick_feed();
  refresh_state();
}

void planner_request_move_by(float mm) {
  float dest = (float)g_pos_steps / g_spmm + mm;
  planner_request_move_to(dest);
}

void planner_request_jog(int dir) {
  float span = config_get()->max_speed_mm_s * 10.0f;
  planner_request_move_by(dir >= 0 ? span : -span);
}

void planner_request_stop(void) {
  if (g_st.homing) {
    g_home_phase = HOME_IDLE;
    g_st.homing = false;
    home_restore_speeds();
  }
  g_stopping = true;
  g_dir_pause = false;
  g_dir_pause_s = 0.0f;
  g_st.has_target = false;
  g_target_steps = g_pos_steps;
  begin_ramp(0.0f);
  if (!g_st.moving) {
    g_vel_mm_s = 0.0f;
    pio_step_stop_soft();
  } else {
    g_st.moving = true;
    pio_step_kick_feed();
  }
  refresh_state();
}

void planner_request_halt(void) { planner_halt(); }

void planner_request_home(void) {
  if (!g_st.enabled || g_st.drv_error) {
    return;
  }
  const McConfig *c = config_get();
  if (c->home_mode == 0 || !home_cfg_ok()) {
    return;
  }

  g_spmm = c->steps_per_mm;
  if (g_spmm < 1e-3f) {
    g_spmm = 1.0f;
  }

  if (!g_home_speeds_saved) {
    g_home_saved_cruise = g_cruise_mm_s;
    g_home_saved_accel = g_accel_mm_s2;
    g_home_speeds_saved = true;
  }
  g_cruise_mm_s = c->home_speed_mm_s;
  g_accel_mm_s2 = c->home_accel_mm_s2;
  if (g_cruise_mm_s < 0.001f) {
    g_cruise_mm_s = 0.001f;
  }
  if (g_accel_mm_s2 < 0.001f) {
    g_accel_mm_s2 = 0.001f;
  }

  g_stopping = false;
  g_st.homing = true;
  g_home_sign = home_seek_sign();

  if (home_ref_asserted()) {
    home_begin_backoff();
  } else if (g_hl_l_stable || g_hl_l_latched) {
    home_begin_clear(+1);
  } else if (g_hl_r_stable || g_hl_r_latched) {
    home_begin_clear(-1);
  } else {
    home_begin_seek();
  }
  refresh_state();
}

void planner_soft_reset(void) {
  g_st.drv_error = false;
  g_drv_err_stable = false;
  g_drv_err_timer_s = 0.0f;
  /* Keep seeded; if pin still asserted, debounce will re-enter error + halt. */
  g_stopping = false;
  g_st.moving = false;
  g_st.homing = false;
  g_home_phase = HOME_IDLE;
  home_restore_speeds();
  g_vel_mm_s = 0.0f;
  g_target_steps = g_pos_steps;
  g_hl_l_latched = false;
  g_hl_r_latched = false;
  reset_ramp();
  pio_step_stop_hard();
  refresh_state();
}

bool planner_is_busy(void) { return g_st.moving || g_st.homing; }
bool planner_is_moving(void) { return g_st.moving; }

bool planner_feed_active(void) {
  return g_st.moving && !g_dir_pause && !g_st.drv_error && g_st.enabled;
}

void planner_get_status(McStatus *out) {
  refresh_state();
  if (out) {
    *out = g_st;
  }
}

#ifndef HOST_TEST

#include "motion_api.h"

void motion_init(void) {
  static bool once;
  if (once) {
    return;
  }
  once = true;
  config_init_defaults();
  planner_init();
}

bool motion_enable(bool on) {
  g_st.enabled = on;
  if (!on) {
    planner_request_stop();
    pio_step_stop_hard();
    g_st.moving = false;
    g_vel_mm_s = 0.0f;
    apply_en_output(false);
  } else {
    apply_en_output(true);
  }
  refresh_state();
  return true;
}

bool motion_move_to(float mm) {
  if (!g_st.enabled) {
    return false;
  }
  const McConfig *c = config_get();
  if ((!isnan(c->slider_min_mm) && mm < c->slider_min_mm) ||
      (!isnan(c->slider_max_mm) && mm > c->slider_max_mm)) {
    g_st.at_soft_limit = true;
    return false;
  }
  float pos_mm = (float)g_pos_steps / g_spmm;
  int sign = (mm > pos_mm + 1e-6f) ? 1 : (mm < pos_mm - 1e-6f) ? -1 : 0;
  if (planner_hard_limit_blocks_sign(sign)) {
    return false;
  }
  planner_request_move_to(mm);
  return true;
}

bool motion_move_by(float mm) {
  if (!g_st.enabled) {
    return false;
  }
  if (planner_hard_limit_blocks_sign(mm < 0.0f ? -1 : (mm > 0.0f ? 1 : 0))) {
    return false;
  }
  planner_request_move_by(mm);
  return true;
}

bool motion_jog(int dir) {
  if (!g_st.enabled) {
    return false;
  }
  if (planner_hard_limit_blocks_sign(dir >= 0 ? 1 : -1)) {
    return false;
  }
  planner_request_jog(dir);
  return true;
}

bool motion_stop(void) {
  planner_request_stop();
  return true;
}

bool motion_halt(void) {
  planner_request_halt();
  return true;
}

bool motion_home(void) {
  if (!g_st.enabled) {
    return false;
  }
  if (config_get()->home_mode == 0) {
    return true;
  }
  if (!home_cfg_ok()) {
    return false;
  }
  planner_request_home();
  return true;
}

bool motion_soft_reset(void) {
  planner_soft_reset();
  return true;
}

bool motion_set_speed(float mm_s) {
  session_get()->speed_mm_s = mm_s;
  g_cruise_mm_s = mm_s;
  return true;
}

bool motion_set_accel(float mm_s2) {
  session_get()->accel_mm_s2 = mm_s2;
  g_accel_mm_s2 = mm_s2;
  return true;
}

bool motion_set_max_speed(float mm_s) {
  config_get()->max_speed_mm_s = mm_s;
  return true;
}

bool motion_set_soft_limits(bool min_en, float min_mm, bool max_en, float max_mm) {
  McConfig *c = config_get();
  c->slider_min_mm = min_en ? min_mm : NAN;
  c->slider_max_mm = max_en ? max_mm : NAN;
  return true;
}

void motion_get_status(McStatus *out) { planner_get_status(out); }

bool motion_is_busy(void) { return planner_is_busy(); }

void motion_stub_tick_ms(unsigned ms) { (void)ms; }

#endif /* !HOST_TEST */
