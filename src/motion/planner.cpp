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
#include "motion_path.h"

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#ifndef HOST_TEST
#include <Arduino.h>
#endif

#include "axis_hw.h"

#define HARD_LIMIT_DEBOUNCE_S 0.020f
#define AXIS_MAX 2

typedef enum {
  HOME_IDLE = 0,
  HOME_CLEAR,
  HOME_SEEK,
  HOME_BACKOFF_REL,
  HOME_BACKOFF_OUT
} HomePhase;

typedef struct {
  int id;
  McStatus st;
  float cruise_mm_s;
  float accel_mm_s2;
  int64_t pos_steps;
  int64_t target_steps;
  float vel_mm_s;
  float ramp_v0;
  float ramp_v1;
  float ramp_phi;
  bool ramp_active;
  bool stopping;
  bool dir_pause;
  float dir_pause_s;
  int last_sign;
  bool fill_wants_more;
  float acc_meas;
  bool braking;
  int brake_d;
  int64_t brake_pos0;
  float brake_v0;
  float spmm;
  bool hl_l_latched;
  bool hl_r_latched;
  bool hl_l_stable;
  bool hl_r_stable;
  float hl_l_timer_s;
  float hl_r_timer_s;
  bool hl_l_raw_prev;
  bool hl_r_raw_prev;
  bool home_stable;
  float home_timer_s;
  bool home_raw_prev;
  bool drv_err_stable;
  float drv_err_timer_s;
  bool drv_err_raw_prev;
  bool drv_err_seeded;
  HomePhase home_phase;
  int home_sign;
  int64_t home_seek_start_pos;
  int64_t home_max_travel_steps;
  float home_saved_cruise;
  float home_saved_accel;
  bool home_speeds_saved;
} PlannerAxis;

static PlannerAxis g_ax[AXIS_MAX];
static bool g_enabled;
static bool g_drv_error;
/* Dual-MT time sync: |d2|/|d1| kept for mid-move SS/SA while coordinated. */
static bool g_coord_active;
static float g_coord_ratio;
static bool g_joy_active;
static float g_joy_pct[AXIS_MAX];

#define AX (g_ax[axis])

static void reset_ramp(int axis);
static void begin_ramp(int axis, float v_cmd);
static void home_restore_speeds(int axis);
static void home_begin_seek(int axis);
static void home_begin_backoff(int axis);
static void home_begin_clear(int axis, int clear_sign);
static void home_finish(int axis);
static void home_abort(int axis, const char *msg);
static void home_start_move_sign(int axis, int sign, float dist_mm);
static bool home_ref_asserted(int axis);
static int home_seek_sign(int axis);
static bool home_cfg_ok(int axis);
static void home_poll_fsm(int axis);
static void planner_halt_axis(int axis);
static void planner_halt_all(void);
static void coord_clear(void);
static void coord_clear_if_idle(void);
static void apply_session_cruise_accel(void);
static void joy_clear(void);
static float cruise_cap(int axis);
static void apply_joy_cruise_accel(void);
static void planner_request_stop_axis(int axis);
static void planner_request_joy(int axis, float signed_v);
static int axis_count(void) { return config_axis2_enabled() ? 2 : 1; }

static void coord_clear(void) {
  g_coord_active = false;
  g_coord_ratio = 1.0f;
}

static void joy_clear(void) {
  g_joy_active = false;
  g_joy_pct[0] = 0.0f;
  g_joy_pct[1] = 0.0f;
}

static void coord_clear_if_idle(void) {
  if (!g_coord_active) {
    return;
  }
  for (int axis = 0; axis < axis_count(); ++axis) {
    if (AX.st.moving || AX.st.homing) {
      return;
    }
  }
  coord_clear();
}

static void scale_cruise_accel(float v0, float a0, float ratio, float *v_out, float *a_out) {
  float v1 = v0 * ratio;
  float a1 = a0 * ratio;
  float vmax = axis_hw_max_speed(1);
  float amax = axis_hw_max_accel(1);
  if (v1 > vmax) {
    v1 = vmax;
  }
  if (a1 > amax) {
    a1 = amax;
  }
  if (v1 < 0.001f) {
    v1 = 0.001f;
  }
  if (a1 < 0.001f) {
    a1 = 0.001f;
  }
  *v_out = v1;
  *a_out = a1;
}

static float clamp_speed_axis(int axis, float v) {
  float mx = axis_hw_max_speed(axis);
  if (v > mx) {
    v = mx;
  }
  if (v < 0.0f) {
    v = 0.0f;
  }
  return v;
}

static float clamp_accel_axis(int axis, float a) {
  float mx = axis_hw_max_accel(axis);
  if (a > mx) {
    a = mx;
  }
  if (a < 0.001f) {
    a = 0.001f;
  }
  return a;
}

/** Apply session (or explicit) cruise/accel: axis0 = (v0,a0); axis1 scaled if coordinated. */
static void apply_cruise_accel(float v0, float a0) {
  g_ax[0].cruise_mm_s = clamp_speed_axis(0, v0);
  g_ax[0].accel_mm_s2 = clamp_accel_axis(0, a0);
  if (g_coord_active && config_axis2_enabled()) {
    float v1, a1;
    scale_cruise_accel(v0, a0, g_coord_ratio, &v1, &a1);
    g_ax[1].cruise_mm_s = v1;
    g_ax[1].accel_mm_s2 = a1;
    return;
  }
  for (int axis = 1; axis < axis_count(); ++axis) {
    AX.cruise_mm_s = clamp_speed_axis(axis, v0);
    AX.accel_mm_s2 = clamp_accel_axis(axis, a0);
  }
}

static void apply_session_cruise_accel(void) {
  apply_cruise_accel(session_get()->speed_mm_s, session_get()->accel_mm_s2);
}

static bool read_limit_raw(int axis, bool left) {
#ifndef HOST_TEST
  if (left) {
    if (!axis_hw_limit_l_use(axis)) {
      return false;
    }
    int level = digitalRead(axis_hw_limit_l_pin(axis)) ? 1 : 0;
    return config_pin_asserted(level, axis_hw_limit_l_active(axis));
  }
  if (!axis_hw_limit_r_use(axis)) {
    return false;
  }
  int level = digitalRead(axis_hw_limit_r_pin(axis)) ? 1 : 0;
  return config_pin_asserted(level, axis_hw_limit_r_active(axis));
#else
  (void)axis;
  (void)left;
  return false;
#endif
}

static bool read_home_raw(int axis) {
#ifndef HOST_TEST
  if (!axis_hw_home_use(axis)) {
    return false;
  }
  int level = digitalRead(axis_hw_home_pin(axis)) ? 1 : 0;
  return config_pin_asserted(level, axis_hw_home_active(axis));
#else
  (void)axis;
  return false;
#endif
}

static bool read_drv_error_raw(int axis) {
#ifndef HOST_TEST
  int level = digitalRead(axis_hw_error_pin(axis)) ? 1 : 0;
  return config_pin_asserted(level, axis_hw_error_active(axis));
#else
  (void)axis;
  return false;
#endif
}

static void apply_en_output(bool enabled) {
#ifndef HOST_TEST
  for (int a = 0; a < axis_count(); ++a) {
    int pin = axis_hw_en_pin(a);
    int active = axis_hw_en_active(a);
    if (enabled) {
      digitalWrite(pin, active ? HIGH : LOW);
    } else {
      digitalWrite(pin, active ? LOW : HIGH);
    }
  }
#else
  (void)enabled;
#endif
}

static void refresh_state(int axis) {
  AX.st.hard_limit = AX.hl_l_latched || AX.hl_r_latched || AX.hl_l_stable || AX.hl_r_stable;
  AX.st.drv_error = g_drv_error;
  AX.st.enabled = g_enabled;
  if (g_drv_error) {
    AX.st.state = MC_STATE_ERROR;
  } else if (AX.st.hard_limit && !AX.st.moving) {
    AX.st.state = MC_STATE_HARD_LIMIT;
  } else if (!g_enabled) {
    AX.st.state = MC_STATE_DISABLED;
  } else if (AX.st.homing) {
    AX.st.state = MC_STATE_HOMING;
  } else if (AX.st.moving) {
    if (AX.braking || AX.stopping) {
      AX.st.state = MC_STATE_DECELERATING;
    } else if (AX.ramp_active) {
      /* Mid-move SS change and start/stop ramps share begin_ramp(axis, v0→v1). */
      if (fabsf(AX.ramp_v1) + 1e-4f < fabsf(AX.ramp_v0)) {
        AX.st.state = MC_STATE_DECELERATING;
      } else if (fabsf(AX.ramp_v1) > fabsf(AX.ramp_v0) + 1e-4f) {
        AX.st.state = MC_STATE_ACCELERATING;
      } else {
        AX.st.state = MC_STATE_MOVING;
      }
    } else {
      AX.st.state = MC_STATE_MOVING;
    }
  } else {
    AX.st.state = MC_STATE_IDLE;
  }
  AX.st.pos_mm = (float)AX.pos_steps / AX.spmm;
  AX.st.target_mm = (float)AX.target_steps / AX.spmm;
  AX.st.vel_mm_s = AX.vel_mm_s;
  /* Measured |a| of the issued words; 0 in cruise/idle. */
  AX.st.acc_mm_s2 = AX.st.moving ? fabsf(AX.acc_meas) : 0.0f;
  AX.st.has_target = AX.st.moving && !AX.stopping;
  if (axis == 0) {
    dbg_hw_set(PIN_DBG_MOV, AX.st.moving || AX.st.homing);
  }
}

/**
 * Emergency halt: immediate STEP abort, EN off, cancel waits/chain.
 * Used by HT/H, hard-limit trips, and PIN_DRV_ERROR.
 */
static void planner_halt_axis(int axis) {
  AX.vel_mm_s = 0.0f;
  AX.st.moving = false;
  AX.st.homing = false;
  AX.home_phase = HOME_IDLE;
  home_restore_speeds(axis);
  AX.stopping = false;
  AX.dir_pause = false;
  AX.st.has_target = false;
  AX.target_steps = AX.pos_steps;
  reset_ramp(axis);
  pio_step_stop_hard(axis);
  refresh_state(axis);
}

static void planner_halt_all(void) {
  protocol_debug(3, "D:halt\n");
  joy_clear();
  coord_clear();
  for (int a = 0; a < axis_count(); ++a) {
    planner_halt_axis(a);
  }
  g_enabled = false;
  apply_en_output(false);
  protocol_cancel_waits_and_chain();
}

static void planner_trip_hard_limit(int axis, bool left) {
  if (AX.st.homing) {
    int hm = axis_hw_home_mode(axis);
    bool is_ref = (hm == 3 && left) || (hm == 4 && !left);
    if (is_ref && AX.home_phase == HOME_SEEK) {
      /* Reference limit hit — enter backoff, do not fault. */
      home_begin_backoff(axis);
      return;
    }
    if (is_ref && (AX.home_phase == HOME_BACKOFF_REL || AX.home_phase == HOME_BACKOFF_OUT)) {
      return;
    }
    if (hm == 1 || hm == 2) {
      protocol_error("home", "hard");
      /* fall through to hard-stop fault */
    }
  }

  if (left) {
    if (AX.hl_l_latched) {
      return;
    }
    AX.hl_l_latched = true;
  } else {
    if (AX.hl_r_latched) {
      return;
    }
    AX.hl_r_latched = true;
  }
  planner_halt_all();
}

bool planner_hard_limit_blocks_sign(int axis, int sign) {
  if (sign < 0 && (AX.hl_l_latched || AX.hl_l_stable)) {
    return true;
  }
  if (sign > 0 && (AX.hl_r_latched || AX.hl_r_stable)) {
    return true;
  }
  return false;
}

bool planner_home_switch_asserted(int axis) {
  return AX.home_stable;
}

bool planner_home_switch_enabled(int axis) {
  return axis_hw_home_use(axis) != 0;
}

static void debounce_side(int axis, bool left, float dt_s) {
  bool use = left ? axis_hw_limit_l_use(axis) : axis_hw_limit_r_use(axis);
  bool *stable = left ? &AX.hl_l_stable : &AX.hl_r_stable;
  bool *latched = left ? &AX.hl_l_latched : &AX.hl_r_latched;
  float *timer = left ? &AX.hl_l_timer_s : &AX.hl_r_timer_s;
  bool *raw_prev = left ? &AX.hl_l_raw_prev : &AX.hl_r_raw_prev;

  if (!use) {
    *stable = false;
    *latched = false;
    *timer = 0.0f;
    *raw_prev = false;
    return;
  }

#ifndef HOST_TEST
  /* Lazy init if CS enables use after boot. */
  pinMode(left ? axis_hw_limit_l_pin(axis) : axis_hw_limit_r_pin(axis), INPUT_PULLUP);
#endif

  bool raw = read_limit_raw(axis, left);
  if (raw != *raw_prev) {
    *timer = 0.0f;
    *raw_prev = raw;
  } else {
    *timer += dt_s;
  }

  if (*timer >= HARD_LIMIT_DEBOUNCE_S) {
    if (raw && !*stable) {
      *stable = true;
      planner_trip_hard_limit(axis, left);
    } else if (!raw && *stable) {
      *stable = false;
      *latched = false;
    }
  }
}

static void debounce_home(int axis, float dt_s) {
  if (!axis_hw_home_use(axis)) {
    AX.home_stable = false;
    AX.home_timer_s = 0.0f;
    AX.home_raw_prev = false;
    return;
  }

#ifndef HOST_TEST
  pinMode(axis_hw_home_pin(axis), INPUT_PULLUP);
#endif

  bool raw = read_home_raw(axis);
  if (raw != AX.home_raw_prev) {
    AX.home_timer_s = 0.0f;
    AX.home_raw_prev = raw;
  } else {
    AX.home_timer_s += dt_s;
  }

  if (AX.home_timer_s >= HARD_LIMIT_DEBOUNCE_S) {
    AX.home_stable = raw;
  }
}

static void debounce_drv_error(int axis, float dt_s) {
#ifndef HOST_TEST
  pinMode(axis_hw_error_pin(axis), INPUT_PULLUP);
#endif

  bool raw = read_drv_error_raw(axis);
  if (!AX.drv_err_seeded) {
    /* Power-up: seed from first sample so an already-asserted pin can become stable. */
    AX.drv_err_raw_prev = raw;
    AX.drv_err_timer_s = 0.0f;
    AX.drv_err_seeded = true;
  } else if (raw != AX.drv_err_raw_prev) {
    AX.drv_err_timer_s = 0.0f;
    AX.drv_err_raw_prev = raw;
  } else {
    AX.drv_err_timer_s += dt_s;
  }

  if (AX.drv_err_timer_s >= HARD_LIMIT_DEBOUNCE_S) {
    if (raw && !AX.drv_err_stable) {
      AX.drv_err_stable = true;
      g_drv_error = true;
      AX.st.drv_error = true;
      planner_halt_all();
    } else if (!raw && AX.drv_err_stable) {
      AX.drv_err_stable = false;
      /* clear only if no axis still asserts; rechecked below */
      bool any = false;
      for (int a = 0; a < axis_count(); ++a) {
        if (g_ax[a].drv_err_stable) { any = true; break; }
      }
      g_drv_error = any;
      AX.st.drv_error = g_drv_error;
      refresh_state(axis);
    }
  }
}

static void planner_poll_switches(int axis, float dt_s) {
  debounce_side(axis, true, dt_s);
  debounce_side(axis, false, dt_s);
  debounce_home(axis, dt_s);
  debounce_drv_error(axis, dt_s);
}

static int64_t mm_to_steps(int axis, float mm) {
  return (int64_t)lroundf(mm * AX.spmm);
}

static void home_restore_speeds(int axis) {
  if (!AX.home_speeds_saved) {
    return;
  }
  AX.cruise_mm_s = AX.home_saved_cruise;
  AX.accel_mm_s2 = AX.home_saved_accel;
  AX.home_speeds_saved = false;
}

static int home_seek_sign(int axis) {
  int hm = axis_hw_home_mode(axis);
  return (hm == 1 || hm == 3) ? -1 : 1;
}

static bool home_ref_asserted(int axis) {
  int hm = axis_hw_home_mode(axis);
  if (hm == 1 || hm == 2) {
    return AX.home_stable;
  }
  if (hm == 3) {
    return AX.hl_l_stable;
  }
  if (hm == 4) {
    return AX.hl_r_stable;
  }
  return false;
}

static bool home_cfg_ok(int axis) {
  switch (axis_hw_home_mode(axis)) {
  case 1:
  case 2:
    return axis_hw_home_use(axis) != 0;
  case 3:
    return axis_hw_limit_l_use(axis) != 0;
  case 4:
    return axis_hw_limit_r_use(axis) != 0;
  default:
    return false;
  }
}

static void home_start_move_sign(int axis, int sign, float dist_mm) {
  if (sign == 0 || dist_mm < 0.0f) {
    return;
  }
  AX.spmm = axis_hw_steps_per_unit(axis);
  if (AX.spmm < 1e-3f) {
    AX.spmm = 1.0f;
  }
  int64_t delta = mm_to_steps(axis, dist_mm);
  if (delta < 1) {
    delta = 1;
  }
  AX.target_steps = AX.pos_steps + (int64_t)sign * delta;
  AX.stopping = false;
  AX.st.moving = true;
  AX.st.has_target = true;
  begin_ramp(axis, AX.vel_mm_s);
  AX.fill_wants_more = false;
  AX.acc_meas = 0.0f;
  AX.braking = false;
  AX.brake_d = 0;
  pio_step_clear_stall(axis); /* stale flag from the previous idle gap */
  pio_step_start(axis);
  pio_step_kick_feed();
}

static void home_begin_seek(int axis) {
  AX.home_phase = HOME_SEEK;
  AX.home_sign = home_seek_sign(axis);
  AX.home_seek_start_pos = AX.pos_steps;
  float span_mm;
  float smin = axis_hw_slider_min(axis);
  float smax = axis_hw_slider_max(axis);
  if (!isnan(smin) && !isnan(smax) && smax > smin) {
    span_mm = smax - smin;
  } else {
    span_mm = CFG_DEFAULT_SLIDER_MAX_MM - CFG_DEFAULT_SLIDER_MIN_MM;
  }
  float max_mm = span_mm * 1.1f;
  if (max_mm < 1.0f) {
    max_mm = 1.0f;
  }
  AX.home_max_travel_steps = mm_to_steps(axis, max_mm);
  if (AX.home_max_travel_steps < 1) {
    AX.home_max_travel_steps = 1;
  }
  home_start_move_sign(axis, AX.home_sign, max_mm);
}

static void home_begin_backoff(int axis) {
  AX.home_phase = HOME_BACKOFF_REL;
  /* Leave reference switch; clear latches on reference limit so drive-out works. */
  int hm = axis_hw_home_mode(axis);
  if (hm == 3) {
    AX.hl_l_latched = false;
  } else if (hm == 4) {
    AX.hl_r_latched = false;
  }
  float span = 50.0f;
  float smin = axis_hw_slider_min(axis);
  float smax = axis_hw_slider_max(axis);
  if (!isnan(smin) && !isnan(smax) && smax > smin) {
    span = (smax - smin) * 0.25f;
  }
  if (span < 10.0f) {
    span = 10.0f;
  }
  home_start_move_sign(axis, -AX.home_sign, span);
}

static void home_begin_clear(int axis, int clear_sign) {
  AX.home_phase = HOME_CLEAR;
  if (clear_sign < 0) {
    AX.hl_l_latched = false;
  } else if (clear_sign > 0) {
    AX.hl_r_latched = false;
  }
  float span = 50.0f;
  float smin = axis_hw_slider_min(axis);
  float smax = axis_hw_slider_max(axis);
  if (!isnan(smin) && !isnan(smax) && smax > smin) {
    span = (smax - smin) * 0.25f;
  }
  if (span < 10.0f) {
    span = 10.0f;
  }
  home_start_move_sign(axis, clear_sign, span);
}

static void home_finish(int axis) {
  float pos_mm;
  float smin = axis_hw_slider_min(axis);
  float smax = axis_hw_slider_max(axis);
  if (axis_hw_home_mode(axis) == 1 || axis_hw_home_mode(axis) == 3) {
    pos_mm = isnan(smin) ? 0.0f : smin;
  } else {
    pos_mm = isnan(smax) ? 0.0f : smax;
  }
  AX.spmm = axis_hw_steps_per_unit(axis);
  if (AX.spmm < 1e-3f) {
    AX.spmm = 1.0f;
  }
  AX.pos_steps = mm_to_steps(axis, pos_mm);
  AX.target_steps = AX.pos_steps;
  AX.vel_mm_s = 0.0f;
  AX.st.moving = false;
  AX.st.homing = false;
  AX.st.has_target = false;
  AX.stopping = false;
  AX.home_phase = HOME_IDLE;
  reset_ramp(axis);
  home_restore_speeds(axis);
  pio_step_stop_soft(axis);
  refresh_state(axis);
}

static void home_abort(int axis, const char *msg) {
  AX.home_phase = HOME_IDLE;
  AX.st.homing = false;
  AX.st.moving = false;
  AX.st.has_target = false;
  AX.stopping = false;
  AX.dir_pause = false;
  AX.vel_mm_s = 0.0f;
  AX.target_steps = AX.pos_steps;
  reset_ramp(axis);
  home_restore_speeds(axis);
  pio_step_stop_hard(axis);
  protocol_error("home", msg ? msg : "abort");
  protocol_cancel_waits_and_chain();
  refresh_state(axis);
}

static void home_poll_fsm(int axis) {
  if (!AX.st.homing || AX.home_phase == HOME_IDLE || AX.stopping) {
    return;
  }

  if (AX.home_phase == HOME_CLEAR) {
    if (!AX.hl_l_stable && !AX.hl_r_stable) {
      AX.hl_l_latched = false;
      AX.hl_r_latched = false;
      AX.vel_mm_s = 0.0f;
      AX.target_steps = AX.pos_steps;
      reset_ramp(axis);
      pio_step_stop_hard(axis);
      if (home_ref_asserted(axis)) {
        home_begin_backoff(axis);
      } else {
        home_begin_seek(axis);
      }
    }
    return;
  }

  if (AX.home_phase == HOME_SEEK) {
    int64_t traveled = AX.pos_steps - AX.home_seek_start_pos;
    if (traveled < 0) {
      traveled = -traveled;
    }
    if (traveled >= AX.home_max_travel_steps) {
      home_abort(axis, "travel");
      return;
    }
    if (home_ref_asserted(axis)) {
      home_begin_backoff(axis);
    }
    return;
  }

  if (AX.home_phase == HOME_BACKOFF_REL) {
    if (!home_ref_asserted(axis)) {
      float out = axis_hw_home_move_out(axis);
      if (out < 0.0f) {
        out = 0.0f;
      }
      AX.home_phase = HOME_BACKOFF_OUT;
      if (out < 1e-4f) {
        home_finish(axis);
      } else {
        home_start_move_sign(axis, -AX.home_sign, out);
      }
    }
    return;
  }

  if (AX.home_phase == HOME_BACKOFF_OUT) {
    if (AX.pos_steps == AX.target_steps && fabsf(AX.vel_mm_s) < 0.01f && pio_step_tx_empty(axis)) {
      home_finish(axis);
    }
  }
}

static void reset_ramp(int axis) {
  AX.ramp_active = false;
  AX.ramp_v0 = AX.vel_mm_s;
  AX.ramp_v1 = AX.vel_mm_s;
  AX.ramp_phi = 0.0f;
}

static void begin_ramp(int axis, float v_cmd) {
  if (!AX.ramp_active || fabsf(v_cmd - AX.ramp_v1) > 1e-4f) {
    AX.ramp_v0 = AX.vel_mm_s;
    AX.ramp_v1 = v_cmd;
    AX.ramp_phi = 0.0f;
    AX.ramp_active = true;
  }
}

static int64_t soft_min_steps(int axis) {
  float mn = axis_hw_window_min(axis);
  if (isnan(mn)) {
    return INT64_MIN / 4;
  }
  return mm_to_steps(axis, mn);
}

static int64_t soft_max_steps(int axis) {
  float mx = axis_hw_window_max(axis);
  if (isnan(mx)) {
    return INT64_MAX / 4;
  }
  return mm_to_steps(axis, mx);
}

static int remaining_steps_for_sign(int axis, int sign) {
  int64_t rem_tgt = AX.target_steps - AX.pos_steps;
  if (sign > 0 && rem_tgt < 0) {
    rem_tgt = 0;
  }
  if (sign < 0 && rem_tgt > 0) {
    rem_tgt = 0;
  }
  int rem = (int)(rem_tgt >= 0 ? rem_tgt : -rem_tgt);

  /* Soft limits do not constrain homing moves. */
  if (!AX.st.homing) {
    if (sign > 0) {
      int64_t to_lim = soft_max_steps(axis) - AX.pos_steps;
      if (to_lim < 0) {
        to_lim = 0;
      }
      if (to_lim < rem) {
        rem = (int)to_lim;
      }
    } else if (sign < 0) {
      int64_t to_lim = AX.pos_steps - soft_min_steps(axis);
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

static float cruise_cap(int axis) {
  float v = AX.cruise_mm_s;
  float mx = axis_hw_max_speed(axis);
  if (v > mx) {
    v = mx;
  }
  float hz_max = planner_max_step_hz(pio_step_sysclk_hz(), PIO_STEP_PERIOD_FIXED);
  float v_hz = hz_max / AX.spmm;
  if (v > v_hz) {
    v = v_hz;
  }
  return v;
}

static float signed_cruise_from_pct(int axis, float pct) {
  if (fabsf(pct) < 1e-3f) {
    return 0.0f;
  }
  float v = fabsf(pct) * 0.01f * session_get()->speed_mm_s;
  v = clamp_speed_axis(axis, v);
  return (pct < 0.0f) ? -v : v;
}

static void apply_joy_cruise_accel(void) {
  for (int axis = 0; axis < axis_count(); ++axis) {
    AX.accel_mm_s2 = clamp_accel_axis(axis, session_get()->accel_mm_s2);
    float pct = g_joy_pct[axis];
    if (fabsf(pct) < 1e-3f) {
      continue;
    }
    float signed_v = signed_cruise_from_pct(axis, pct);
    AX.cruise_mm_s = fabsf(signed_v);
    if (AX.st.moving && !AX.stopping) {
      int sign = (signed_v >= 0.0f) ? 1 : -1;
      begin_ramp(axis, (float)sign * cruise_cap(axis));
    }
  }
}

static void settle_if_done(int axis) {
  int64_t err = AX.target_steps - AX.pos_steps;
  if (AX.stopping) {
    if (fabsf(AX.vel_mm_s) < 0.01f && pio_step_tx_empty(axis)) {
      AX.vel_mm_s = 0.0f;
      AX.st.moving = false;
      AX.st.homing = false;
      AX.home_phase = HOME_IDLE;
      home_restore_speeds(axis);
      AX.stopping = false;
      AX.target_steps = AX.pos_steps;
      reset_ramp(axis);
      pio_step_stop_soft(axis);
      refresh_state(axis);
      coord_clear_if_idle();
    }
    return;
  }
  if (AX.st.homing && AX.home_phase != HOME_IDLE) {
    /* Homing FSM owns completion (home_poll_fsm / home_finish). */
    return;
  }
  if (err == 0 && fabsf(AX.vel_mm_s) < 0.01f) {
    AX.vel_mm_s = 0.0f;
    AX.st.moving = false;
    AX.st.homing = false;
    reset_ramp(axis);
    if (pio_step_tx_empty(axis)) {
      pio_step_stop_soft(axis);
    }
    refresh_state(axis);
    coord_clear_if_idle();
  }
}

void planner_init(void) {
  g_enabled = false;
  g_drv_error = false;
  coord_clear();
  joy_clear();
  for (int axis = 0; axis < AXIS_MAX; ++axis) {
    memset(&AX, 0, sizeof(AX));
    AX.id = axis;
    AX.st.state = MC_STATE_DISABLED;
    AX.spmm = axis_hw_steps_per_unit(axis);
    if (AX.spmm < 1e-3f) {
      AX.spmm = 1.0f;
    }
    AX.cruise_mm_s = session_get()->speed_mm_s;
    AX.accel_mm_s2 = session_get()->accel_mm_s2;
    reset_ramp(axis);
    refresh_state(axis);
  }
  motion_diag_init_from_noinit();
  pio_step_init();
}

void planner_set_cruise_accel(int axis, float cruise_mm_s, float accel_mm_s2) {
  if (axis < 0 || axis >= AXIS_MAX) {
    return;
  }
  AX.cruise_mm_s = cruise_mm_s;
  AX.accel_mm_s2 = accel_mm_s2;
}

int planner_fill_fifo(int axis) {
  int emitted = 0;
  if (!AX.st.moving || g_drv_error || !g_enabled) {
    AX.fill_wants_more = false;
    return 0;
  }
  if (AX.dir_pause) {
    AX.fill_wants_more = false;
    return 0;
  }

  unsigned room_budget = 6;
  while (room_budget-- && pio_step_tx_room(axis) > 0) {
    int64_t err = AX.target_steps - AX.pos_steps;
    int sign;
    if (AX.stopping) {
      if (fabsf(AX.vel_mm_s) < 0.01f) {
        AX.fill_wants_more = false;
        break;
      }
      sign = (AX.vel_mm_s >= 0.0f) ? 1 : -1;
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
      float v_stop = (stop_app_hz > 0) ? (float)stop_app_hz / AX.spmm : 0.0f;
      if (fabsf(AX.vel_mm_s) <= v_stop + 1e-3f ||
          planner_stop_rem_steps(AX.vel_mm_s, AX.accel_mm_s2, AX.spmm) <= 4) {
        AX.vel_mm_s = 0.0f;
        AX.fill_wants_more = false;
        break;
      }
      sign = (AX.vel_mm_s >= 0.0f) ? 1 : -1;
    }

    /* Reverse: decelerate in current travel direction until ~0, then pause. */
    bool reverse_decel = false;
    if (!AX.stopping && planner_needs_reverse_decel(AX.last_sign, sign, AX.vel_mm_s)) {
      begin_ramp(axis, 0.0f);
      sign = (AX.vel_mm_s >= 0.0f) ? 1 : -1;
      reverse_decel = true;
      protocol_debug(4, "D:reverse_decel v=%.2f\n", (double)AX.vel_mm_s);
    }

    int rem;
    if (AX.stopping || err == 0 || reverse_decel) {
      rem = planner_stop_rem_steps(AX.vel_mm_s, AX.accel_mm_s2, AX.spmm);
    } else {
      rem = remaining_steps_for_sign(axis, sign);
    }

    if (rem <= 0) {
      if (err != 0 && !AX.stopping && !reverse_decel) {
        /* Nothing left to issue although the target is not reached (soft limit).
           Drop the target so settle_if_done(axis) can leave the moving state. */
        motion_diag_note_overshoot(1);
        protocol_debug(2, "D:move_blocked pos=%ld tgt=%ld\n", (long)AX.pos_steps,
                       (long)AX.target_steps);
        AX.target_steps = AX.pos_steps;
      }
      AX.vel_mm_s = 0.0f;
      reset_ramp(axis);
      AX.braking = false;
      AX.brake_d = 0;
      AX.fill_wants_more = false;
      break;
    }

    float rem_mm = (float)rem / AX.spmm;
    float vmax = planner_vmax_for_distance(rem_mm, AX.accel_mm_s2);
    bool need_brake = !AX.stopping && !reverse_decel &&
                      rem <= planner_stop_rem_steps(fabsf(AX.vel_mm_s), AX.accel_mm_s2, AX.spmm) + 4;
    /*
     * Ramp target is either cruise or 0. Feeding it min(cruise, vmax) would
     * move the target every word as rem shrinks, restart the sine phase, and
     * leave the axis stuck near ramp_start_hz (seen as "higher ss → slower").
     * When remaining distance cannot support the current speed, brake to 0;
     * the vmax clamp below is the hard safety net for the issued word.
     *
     * Use the true stop-distance threshold here instead of a stale
     * speed-vs-vmax trigger so the brake arc starts at the same geometric
     * point that the accel arc ended, avoiding the visible accel/decel skew.
     */
    float v_cmd;
    if (AX.stopping || reverse_decel) {
      v_cmd = 0.0f;
    } else if (AX.braking || need_brake) {
      /* Braking point reached: commit to one ramp down to 0. Re-deriving it per
         word would restart the sine and flatten it into constant 2a/pi. */
      AX.braking = true;
      v_cmd = 0.0f;
    } else {
      v_cmd = (float)sign * cruise_cap(axis);
    }

    bool decel = AX.stopping || reverse_decel || AX.braking;
    if (decel) {
      /* Commit the brake arc once and then track progress along it by distance.
         A soft-stop rem is re-derived from v every word and would drift; keyed
         on distance the profile lands on the target by construction. */
      if (AX.brake_d <= 0) {
        AX.brake_d = rem;
        AX.brake_pos0 = AX.pos_steps;
        AX.brake_v0 = fabsf(AX.vel_mm_s);
      }
      rem = AX.brake_d - (int)llabs(AX.pos_steps - AX.brake_pos0);
      if (rem <= 0 || AX.brake_v0 <= 0.0f) {
        AX.vel_mm_s = 0.0f;
        reset_ramp(axis);
        AX.braking = false;
        AX.brake_d = 0;
        AX.fill_wants_more = false;
        break;
      }
    } else {
      AX.brake_d = 0;
    }

    /* Direction change at zero crossing */
    if (!AX.stopping && AX.last_sign != 0 && sign != AX.last_sign && fabsf(AX.vel_mm_s) < 0.05f) {
      float pause = config_get()->dir_change_pause_s;
      if (pause > 0.0f) {
        AX.dir_pause = true;
        AX.dir_pause_s = pause;
        AX.vel_mm_s = 0.0f;
        reset_ramp(axis);
        AX.braking = false;
        AX.brake_d = 0;
        AX.fill_wants_more = false;
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
    if (ramp_start > 0 && !AX.stopping && !reverse_decel && !AX.braking) {
      float vmin = (float)ramp_start / AX.spmm;
      if (vmin > 0.0f && fabsf(AX.vel_mm_s) < vmin && fabsf(v_cmd) > vmin) {
        AX.vel_mm_s = (float)sign * vmin;
        reset_ramp(axis);
      }
    }

    begin_ramp(axis, v_cmd);

    /*
     * ramp_start_hz is the *launch* floor only. Applying it while braking pinned
     * STEP at ramp_start for the last d_stop(ramp_start) — the axis then halted
     * from that rate instead of tapering (end-of-move snap). Approaching the
     * target is bounded by stop_approach_hz instead.
     */
    float min_hz = (decel && stop_app > 0) ? (float)stop_app : (float)ramp_start;

    float step_hz_est = fabsf(AX.vel_mm_s) * AX.spmm;
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

    int pending = pio_step_pending_steps(axis);
    int n = planner_pack_n(step_hz_est, rem, pending);
    if (n <= 0) {
      /* Enough steps already queued for the time budget. */
      AX.fill_wants_more = false;
      break;
    }
    if (n > rem) {
      n = rem;
    }
    /*
     * Curvature cap while braking: the time-budget pack size only shrinks
     * once close to the target (rem <= remaining/5), so a word entered mid
     * brake-arc (steepest part of the sine, near D/2) could still hold ~64
     * pulses at one fixed rate — a visible speed "step" each time the next
     * word's rate is applied (reported as pumping / un-smooth deceleration).
     * |dv/dR| peaks at pi*dv_total/(2*D) (at R=D/2); bounding n against that
     * worst case keeps the per-word speed change under ~brake_step_frac of
     * the total brake span everywhere on the arc, not just near the target.
     */
    if (decel && AX.brake_d > 0) {
      const float brake_step_frac = 0.06f;
      int n_curv = (int)(brake_step_frac * (2.0f / (float)M_PI) * (float)AX.brake_d);
      if (n_curv < 1) {
        n_curv = 1;
      }
      if (n > n_curv) {
        n = n_curv;
      }
    }

    /* Compute the actual PIO delay for this word from the start-of-word
       rate estimate and derive the real issued duration (dt) for the packed
       pulses. Reuse that delay for the emitted word so phi advancement
       matches the issued timing. */
    uint32_t delay_cycles_for_word =
        planner_hz_to_delay(step_hz_est, pio_step_sysclk_hz(), PIO_STEP_PERIOD_FIXED);
    /* dt (seconds) = n * period; period = (fixed_overhead + delay_cycles)/sysclk */
    double dt = (double)n * ((double)delay_cycles_for_word + (double)PIO_STEP_PERIOD_FIXED) /
                (double)pio_step_sysclk_hz();
    /* One refinement pass: estimate end-of-word velocity and recompute a
       delay from the average rate, improving alignment between phi advance
       and the issued PIO word without heavy iteration. Cover both the accel
       ramp and the brake arc — omitting decel here left every braking word's
       delay keyed on the start-of-word (pre-decel) rate, i.e. the corner
       from steady speed into deceleration ran a whole word too fast before
       snapping to the next word's slower rate. */
    if (decel && AX.brake_d > 0) {
      float v_app_est = (stop_app > 0) ? (float)stop_app / AX.spmm : 0.0f;
      float dv_est = AX.brake_v0 - v_app_est;
      if (dv_est < 0.0f) {
        dv_est = 0.0f;
      }
      int r_after_est = rem - n;
      float v_end_est;
      if (r_after_est <= 0) {
        v_end_est = v_app_est;
      } else {
        float frac_est = (float)M_PI * (float)r_after_est / (2.0f * (float)AX.brake_d);
        v_end_est = v_app_est + dv_est * sinf(frac_est);
      }
      double step_hz_est2 = fabs((double)v_end_est) * (double)AX.spmm;
      if (step_hz_est2 < 1.0) {
        step_hz_est2 = (double)min_hz;
      }
      uint32_t delay2 = planner_hz_to_delay((float)step_hz_est2, pio_step_sysclk_hz(), PIO_STEP_PERIOD_FIXED);
      double dt2 = (double)n * ((double)delay2 + (double)PIO_STEP_PERIOD_FIXED) / (double)pio_step_sysclk_hz();
      if (fabs(dt2 - dt) / (dt + 1e-12) > 0.02) {
        dt = dt2;
        delay_cycles_for_word = delay2;
      }
    } else if (!decel && AX.ramp_active) {
      double phi_est = planner_sine_advance_phi((double)AX.ramp_phi, AX.ramp_v0, AX.ramp_v1, AX.accel_mm_s2, dt);
      double v_est = (double)planner_sine_vel(AX.ramp_v0, AX.ramp_v1, (float)phi_est);
      double step_hz_est2 = fabs(v_est) * (double)AX.spmm;
      if (step_hz_est2 < 1.0) {
        step_hz_est2 = (double)min_hz;
      }
      uint32_t delay2 = planner_hz_to_delay((float)step_hz_est2, pio_step_sysclk_hz(), PIO_STEP_PERIOD_FIXED);
      double dt2 = (double)n * ((double)delay2 + (double)PIO_STEP_PERIOD_FIXED) / (double)pio_step_sysclk_hz();
      /* Accept the refined dt if it differs noticeably. */
      if (fabs(dt2 - dt) / (dt + 1e-12) > 0.02) {
        dt = dt2;
        delay_cycles_for_word = delay2;
      }
    }
    float v_prev = AX.vel_mm_s;
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
      float v_app = (stop_app > 0) ? (float)stop_app / AX.spmm : 0.0f;
      float dv = AX.brake_v0 - v_app;
      if (dv < 0.0f) {
        dv = 0.0f;
      }
      int r_after = rem - n;
      if (r_after <= 0) {
        AX.vel_mm_s = (float)sign * v_app;
      } else {
        float frac = (float)M_PI * (float)r_after / (2.0f * (float)AX.brake_d);
        AX.vel_mm_s = (float)sign * (v_app + dv * sinf(frac));
      }
    } else {
      AX.ramp_phi = (float)planner_sine_advance_phi((double)AX.ramp_phi, AX.ramp_v0, AX.ramp_v1, AX.accel_mm_s2, dt);
      AX.vel_mm_s = planner_sine_vel(AX.ramp_v0, AX.ramp_v1, AX.ramp_phi);
      if (AX.ramp_phi >= 1.0f) {
        AX.vel_mm_s = AX.ramp_v1;
        AX.ramp_active = false;
      }
    }

    /* Clamp to stop distance. While braking this is only an outer safety cap,
       so the committed ramp is not torn down by it. */
    float v_cap = AX.braking ? vmax * 1.25f : vmax;
    if (fabsf(AX.vel_mm_s) > v_cap) {
      AX.vel_mm_s = (AX.vel_mm_s >= 0.0f) ? v_cap : -v_cap;
      if (!AX.braking) {
        reset_ramp(axis);
      }
    }

    float step_hz = fabsf(AX.vel_mm_s) * AX.spmm;
    if (!decel && rem > 1 && step_hz < (float)ramp_start) {
      step_hz = (float)ramp_start;
    }
    if ((decel || rem <= 4) && stop_app > 0 && step_hz > 0.0f &&
        step_hz < (float)stop_app) {
      step_hz = (float)stop_app;
    }
    if (step_hz < 1.0f) {
      if (AX.stopping || reverse_decel || err == 0) {
        AX.vel_mm_s = 0.0f;
        AX.fill_wants_more = false;
        break;
      }
      step_hz = 1.0f;
    }

    /* Use the previously computed delay (based on start-of-word estimate)
       so the PIO word duration aligns with the dt used above. */
    uint32_t delay = delay_cycles_for_word;
    pio_step_set_dir(axis, sign);
    if (!pio_step_put_word(axis, delay, (uint8_t)n)) {
      /* TX full: work remains, so a dry FIFO later would be a real underrun. */
      AX.fill_wants_more = true;
      break;
    }
    AX.fill_wants_more = true;
    motion_diag_note_hz(step_hz);
    if (dt > 1e-6f) {
      float a_now = (AX.vel_mm_s - v_prev) / dt;
      AX.acc_meas += 0.25f * (a_now - AX.acc_meas);
    }
    int64_t pos_before = AX.pos_steps;
    AX.pos_steps += (int64_t)n * (int64_t)sign;
    AX.last_sign = sign;
    ++emitted;

    /*
     * Overshoot = this word carried the axis across the target. Being *beyond*
     * the target is normal while the trailing deceleration is issued (err == 0)
     * or while a reverse move bleeds off speed, so only a crossing counts.
     */
    if (!AX.stopping && !reverse_decel && err != 0) {
      bool crossed = (sign > 0 && pos_before <= AX.target_steps && AX.pos_steps > AX.target_steps) ||
                     (sign < 0 && pos_before >= AX.target_steps && AX.pos_steps < AX.target_steps);
      if (crossed) {
        motion_diag_note_overshoot((int)llabs(AX.pos_steps - AX.target_steps));
        protocol_debug(2, "D:overshoot %ld\n", (long)llabs(AX.pos_steps - AX.target_steps));
        AX.pos_steps = AX.target_steps;
        AX.vel_mm_s = 0.0f;
        reset_ramp(axis);
        AX.braking = false;
        AX.brake_d = 0;
        AX.fill_wants_more = false;
        break;
      }
    }
  }
  refresh_state(axis);
  return emitted;
}


void planner_tick_axis(int axis, float dt_s) {
  planner_poll_switches(axis, dt_s);
  home_poll_fsm(axis);
  if (AX.dir_pause) {
    AX.dir_pause_s -= dt_s;
    if (AX.dir_pause_s <= 0.0f) {
      AX.dir_pause = false;
      AX.dir_pause_s = 0.0f;
      AX.last_sign = 0;
      pio_step_clear_stall(axis);
    }
  }

  if (AX.st.moving && !AX.dir_pause && AX.fill_wants_more && pio_step_is_stalled(axis) &&
      pio_step_tx_empty(axis)) {
    motion_diag_note_underrun();
    protocol_debug(2, "D:underrun a=%d\n", axis);
#ifndef HOST_TEST
#ifdef DEBUG_HW
    dbg_hw_set(PIN_DBG_UNDERRUN, 1);
    dbg_hw_set(PIN_DBG_UNDERRUN, 0);
#endif
#endif
  }

  if (AX.st.moving) {
    if (!pio_step_tx_empty(axis) || fabsf(AX.vel_mm_s) > 0.01f ||
        AX.pos_steps != AX.target_steps || AX.stopping) {
      pio_step_start(axis);
    }
    settle_if_done(axis);
  }
  home_poll_fsm(axis);
  refresh_state(axis);
}

void planner_tick(float dt_s) {
  if (dt_s < 0.0f) {
    dt_s = 0.0f;
  }
  for (int axis = 0; axis < axis_count(); ++axis) {
    planner_tick_axis(axis, dt_s);
  }
}

static bool clamp_target_mm(int axis, float *mm) {
  float mn = axis_hw_window_min(axis);
  float mx = axis_hw_window_max(axis);
  bool ok = true;
  if (!isnan(mn) && *mm < mn) {
    *mm = mn;
    AX.st.at_soft_limit = true;
    ok = false;
  } else if (!isnan(mx) && *mm > mx) {
    *mm = mx;
    AX.st.at_soft_limit = true;
    ok = false;
  } else {
    AX.st.at_soft_limit = false;
  }
  return ok;
}

void planner_request_move_to(int axis, float mm) {
  if (axis < 0 || axis >= axis_count() || !g_enabled || g_drv_error) {
    return;
  }
  clamp_target_mm(axis, &mm);
  AX.spmm = axis_hw_steps_per_unit(axis);
  float pos_mm = (float)AX.pos_steps / AX.spmm;
  int sign = 0;
  if (mm > pos_mm + 1e-6f) {
    sign = 1;
  } else if (mm < pos_mm - 1e-6f) {
    sign = -1;
  }
  if (planner_hard_limit_blocks_sign(axis, sign)) {
    return;
  }
  AX.target_steps = mm_to_steps(axis, mm);
  AX.stopping = false;
  AX.st.moving = true;
  AX.st.homing = false;
  AX.st.has_target = true;
  begin_ramp(axis, AX.vel_mm_s);
  AX.fill_wants_more = false;
  AX.acc_meas = 0.0f;
  AX.braking = false;
  AX.brake_d = 0;
  pio_step_clear_stall(axis);
  pio_step_start(axis);
  pio_step_kick_feed();
  refresh_state(axis);
}

void planner_request_move_by(int axis, float mm) {
  float dest = (float)AX.pos_steps / AX.spmm + mm;
  planner_request_move_to(axis, dest);
}

void planner_request_jog(int axis, int dir) {
  float span = config_get()->max_speed_mm_s * 10.0f;
  planner_request_move_by(axis, dir >= 0 ? span : -span);
}

static void planner_request_joy(int axis, float signed_v) {
  if (axis < 0 || axis >= axis_count() || !g_enabled || g_drv_error) {
    return;
  }
  AX.spmm = axis_hw_steps_per_unit(axis);
  if (AX.spmm < 1e-3f) {
    AX.spmm = 1.0f;
  }
  AX.accel_mm_s2 = clamp_accel_axis(axis, session_get()->accel_mm_s2);

  if (fabsf(signed_v) < 1e-4f) {
    if (AX.st.moving && !AX.stopping) {
      planner_request_stop_axis(axis);
    }
    return;
  }

  int sign = signed_v > 0.0f ? 1 : -1;
  if (planner_hard_limit_blocks_sign(axis, sign)) {
    if (AX.st.moving && !AX.stopping) {
      planner_request_stop_axis(axis);
    }
    return;
  }

  int64_t dest_steps = (sign > 0) ? soft_max_steps(axis) : soft_min_steps(axis);
  if ((sign > 0 && AX.pos_steps >= dest_steps) || (sign < 0 && AX.pos_steps <= dest_steps)) {
    if (AX.st.moving && !AX.stopping) {
      planner_request_stop_axis(axis);
    }
    return;
  }

  float cruise = fabsf(signed_v);
  bool same = AX.st.moving && !AX.stopping && AX.target_steps == dest_steps &&
              fabsf(AX.cruise_mm_s - cruise) < 1e-4f;
  AX.cruise_mm_s = cruise;
  if (same) {
    return;
  }

  AX.target_steps = dest_steps;
  AX.stopping = false;
  AX.st.moving = true;
  AX.st.homing = false;
  AX.st.has_target = true;
  AX.fill_wants_more = false;
  AX.acc_meas = 0.0f;
  AX.braking = false;
  AX.brake_d = 0;
  begin_ramp(axis, (float)sign * cruise_cap(axis));
  pio_step_clear_stall(axis);
  pio_step_start(axis);
  pio_step_kick_feed();
  refresh_state(axis);
}

static void planner_request_stop_axis(int axis) {
  if (AX.st.homing) {
    AX.home_phase = HOME_IDLE;
    AX.st.homing = false;
    home_restore_speeds(axis);
  }
  AX.stopping = true;
  AX.dir_pause = false;
  AX.dir_pause_s = 0.0f;
  AX.st.has_target = false;
  AX.target_steps = AX.pos_steps;
  begin_ramp(axis, 0.0f);
  if (!AX.st.moving) {
    AX.vel_mm_s = 0.0f;
    pio_step_stop_soft(axis);
  } else {
    AX.st.moving = true;
    pio_step_kick_feed();
  }
  refresh_state(axis);
}

void planner_request_stop(void) {
  for (int axis = 0; axis < axis_count(); ++axis) {
    planner_request_stop_axis(axis);
  }
}

void planner_request_halt(void) { planner_halt_all(); }

void planner_takeover_from_path(int axis, int64_t pos_steps, float vel_mm_s) {
  if (axis < 0 || axis >= AXIS_MAX) {
    return;
  }
  joy_clear();
  AX.pos_steps = pos_steps;
  AX.target_steps = pos_steps;
  AX.vel_mm_s = vel_mm_s;
  AX.last_sign = (vel_mm_s >= 0.0f) ? 1 : -1;
  AX.st.moving = true;
  AX.st.homing = false;
  AX.st.has_target = false;
  reset_ramp(axis);
  refresh_state(axis);
}

void planner_request_home(int axis) {
  if (axis < 0 || axis >= axis_count() || !g_enabled || g_drv_error) {
    return;
  }
  if (axis_hw_home_mode(axis) == 0 || !home_cfg_ok(axis)) {
    return;
  }

  AX.spmm = axis_hw_steps_per_unit(axis);
  if (AX.spmm < 1e-3f) {
    AX.spmm = 1.0f;
  }

  if (!AX.home_speeds_saved) {
    AX.home_saved_cruise = AX.cruise_mm_s;
    AX.home_saved_accel = AX.accel_mm_s2;
    AX.home_speeds_saved = true;
  }
  AX.cruise_mm_s = axis_hw_home_speed(axis);
  AX.accel_mm_s2 = axis_hw_home_accel(axis);
  if (AX.cruise_mm_s < 0.001f) {
    AX.cruise_mm_s = 0.001f;
  }
  if (AX.accel_mm_s2 < 0.001f) {
    AX.accel_mm_s2 = 0.001f;
  }

  AX.stopping = false;
  AX.st.homing = true;
  AX.home_sign = home_seek_sign(axis);

  if (home_ref_asserted(axis)) {
    home_begin_backoff(axis);
  } else if (AX.hl_l_stable || AX.hl_l_latched) {
    home_begin_clear(axis, +1);
  } else if (AX.hl_r_stable || AX.hl_r_latched) {
    home_begin_clear(axis, -1);
  } else {
    home_begin_seek(axis);
  }
  refresh_state(axis);
}

void planner_soft_reset(void) {
  g_drv_error = false;
  joy_clear();
  coord_clear();
  for (int axis = 0; axis < axis_count(); ++axis) {
    AX.st.drv_error = false;
    AX.drv_err_stable = false;
    AX.drv_err_timer_s = 0.0f;
    AX.stopping = false;
    AX.st.moving = false;
    AX.st.homing = false;
    AX.home_phase = HOME_IDLE;
    home_restore_speeds(axis);
    AX.vel_mm_s = 0.0f;
    AX.target_steps = AX.pos_steps;
    AX.hl_l_latched = false;
    AX.hl_r_latched = false;
    reset_ramp(axis);
    pio_step_stop_hard(axis);
    refresh_state(axis);
  }
}

bool planner_is_busy(void) {
  for (int axis = 0; axis < axis_count(); ++axis) {
    if (AX.st.moving || AX.st.homing) {
      return true;
    }
  }
  return false;
}

bool planner_is_moving(void) {
  for (int axis = 0; axis < axis_count(); ++axis) {
    if (AX.st.moving) {
      return true;
    }
  }
  return false;
}

bool planner_feed_active_axis(int axis) {
  if (axis < 0 || axis >= axis_count()) {
    return false;
  }
  return AX.st.moving && !AX.dir_pause && !g_drv_error && g_enabled;
}

bool planner_feed_active(void) {
  for (int axis = 0; axis < axis_count(); ++axis) {
    if (planner_feed_active_axis(axis)) {
      return true;
    }
  }
  return false;
}

static McState merge_state(void) {
  if (g_drv_error) {
    return MC_STATE_ERROR;
  }
  bool any_hl = false, any_homing = false, any_moving = false;
  bool any_accel = false, any_decel = false;
  for (int axis = 0; axis < axis_count(); ++axis) {
    refresh_state(axis);
    if (AX.st.hard_limit && !AX.st.moving) {
      any_hl = true;
    }
    if (AX.st.homing) {
      any_homing = true;
    }
    if (AX.st.moving) {
      any_moving = true;
      if (AX.st.state == MC_STATE_ACCELERATING) {
        any_accel = true;
      }
      if (AX.st.state == MC_STATE_DECELERATING) {
        any_decel = true;
      }
    }
  }
  if (any_hl && !any_moving && !any_homing) {
    return MC_STATE_HARD_LIMIT;
  }
  if (!g_enabled) {
    return MC_STATE_DISABLED;
  }
  if (any_homing) {
    return MC_STATE_HOMING;
  }
  if (any_decel) {
    return MC_STATE_DECELERATING;
  }
  if (any_accel) {
    return MC_STATE_ACCELERATING;
  }
  if (any_moving) {
    return MC_STATE_MOVING;
  }
  return MC_STATE_IDLE;
}

void planner_get_status(McStatus *out) {
  if (!out) {
    return;
  }
  memset(out, 0, sizeof(*out));
  out->enabled = g_enabled;
  out->drv_error = g_drv_error;
  out->state = merge_state();
  out->pos_mm = (float)g_ax[0].pos_steps / (g_ax[0].spmm > 1e-3f ? g_ax[0].spmm : 1.0f);
  out->pos_mm_2 = 0.0f;
  if (config_axis2_enabled()) {
    out->pos_mm_2 = (float)g_ax[1].pos_steps / (g_ax[1].spmm > 1e-3f ? g_ax[1].spmm : 1.0f);
  }
  out->moving = planner_is_moving();
  out->homing = g_ax[0].st.homing || (config_axis2_enabled() && g_ax[1].st.homing);
  out->hard_limit = g_ax[0].st.hard_limit || (config_axis2_enabled() && g_ax[1].st.hard_limit);
  out->at_soft_limit = g_ax[0].st.at_soft_limit;
  out->has_target = g_ax[0].st.has_target;
  out->target_mm = g_ax[0].st.target_mm;
  out->vel_mm_s = g_ax[0].st.vel_mm_s;
  out->acc_mm_s2 = g_ax[0].st.acc_mm_s2;
  out->target_mm_2 = 0.0f;
  out->vel_mm_s_2 = 0.0f;
  out->acc_mm_s2_2 = 0.0f;
  if (config_axis2_enabled()) {
    out->target_mm_2 = g_ax[1].st.target_mm;
    out->vel_mm_s_2 = g_ax[1].st.vel_mm_s;
    out->acc_mm_s2_2 = g_ax[1].st.acc_mm_s2;
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
  (void)config_axis2_enabled();
  planner_init();
  motion_path_init();
}

bool motion_enable(bool on) {
  g_enabled = on;
  for (int axis = 0; axis < axis_count(); ++axis) {
    AX.st.enabled = on;
  }
  if (!on) {
    joy_clear();
    planner_request_stop();
    coord_clear();
    for (int axis = 0; axis < axis_count(); ++axis) {
      pio_step_stop_hard(axis);
      AX.st.moving = false;
      AX.vel_mm_s = 0.0f;
      refresh_state(axis);
    }
    apply_en_output(false);
  } else {
    apply_en_output(true);
    for (int axis = 0; axis < axis_count(); ++axis) {
      refresh_state(axis);
    }
  }
  return true;
}

bool motion_move_to(float mm) {
  if (!g_enabled) {
    return false;
  }
  {
    float mn = axis_hw_window_min(0);
    float mx = axis_hw_window_max(0);
    if ((!isnan(mn) && mm < mn) || (!isnan(mx) && mm > mx)) {
      g_ax[0].st.at_soft_limit = true;
      return false;
    }
  }
  float pos_mm = (float)g_ax[0].pos_steps / g_ax[0].spmm;
  int sign = (mm > pos_mm + 1e-6f) ? 1 : (mm < pos_mm - 1e-6f) ? -1 : 0;
  if (planner_hard_limit_blocks_sign(0, sign)) {
    return false;
  }
  joy_clear();
  coord_clear();
  g_ax[0].cruise_mm_s = session_get()->speed_mm_s;
  g_ax[0].accel_mm_s2 = session_get()->accel_mm_s2;
  planner_request_move_to(0, mm);
  return true;
}

bool motion_move_to2(float mm1_or_nan, float mm2_or_nan) {
  if (!g_enabled) {
    return false;
  }
  bool do0 = !isnan(mm1_or_nan);
  bool do1 = config_axis2_enabled() && !isnan(mm2_or_nan);
  if (!do0 && !do1) {
    return true;
  }
  if (do0) {
    float mn = axis_hw_window_min(0);
    float mx = axis_hw_window_max(0);
    if ((!isnan(mn) && mm1_or_nan < mn) || (!isnan(mx) && mm1_or_nan > mx)) {
      g_ax[0].st.at_soft_limit = true;
      return false;
    }
  }
  if (do1) {
    float mn = axis_hw_window_min(1);
    float mx = axis_hw_window_max(1);
    if ((!isnan(mn) && mm2_or_nan < mn) || (!isnan(mx) && mm2_or_nan > mx)) {
      g_ax[1].st.at_soft_limit = true;
      return false;
    }
  }

  float pos0 = (float)g_ax[0].pos_steps / (g_ax[0].spmm > 1e-3f ? g_ax[0].spmm : 1.0f);
  float pos1 = (float)g_ax[1].pos_steps / (g_ax[1].spmm > 1e-3f ? g_ax[1].spmm : 1.0f);
  float d0 = do0 ? (mm1_or_nan - pos0) : 0.0f;
  float d1 = do1 ? (mm2_or_nan - pos1) : 0.0f;
  if (do0 && fabsf(d0) < 1e-6f) {
    do0 = false;
  }
  if (do1 && fabsf(d1) < 1e-6f) {
    do1 = false;
  }
  if (!do0 && !do1) {
    joy_clear();
    coord_clear();
    return true;
  }

  if (do0 && planner_hard_limit_blocks_sign(0, d0 > 0 ? 1 : -1)) {
    return false;
  }
  if (do1 && planner_hard_limit_blocks_sign(1, d1 > 0 ? 1 : -1)) {
    return false;
  }

  joy_clear();
  /* Coordinated: session speed/accel on axis0; scale axis1 to match duration. */
  if (do0 && do1) {
    g_coord_active = true;
    g_coord_ratio = fabsf(d1) / fabsf(d0);
    apply_session_cruise_accel();
  } else {
    coord_clear();
    float v0 = session_get()->speed_mm_s;
    float a0 = session_get()->accel_mm_s2;
    if (do0) {
      g_ax[0].cruise_mm_s = v0;
      g_ax[0].accel_mm_s2 = a0;
    }
    if (do1) {
      g_ax[1].cruise_mm_s = v0;
      g_ax[1].accel_mm_s2 = a0;
    }
  }

  if (do0) {
    planner_request_move_to(0, mm1_or_nan);
  }
  if (do1) {
    planner_request_move_to(1, mm2_or_nan);
  }
  return true;
}

bool motion_move_by(float mm) {
  if (!g_enabled) {
    return false;
  }
  if (planner_hard_limit_blocks_sign(0, mm < 0.0f ? -1 : (mm > 0.0f ? 1 : 0))) {
    return false;
  }
  joy_clear();
  coord_clear();
  g_ax[0].cruise_mm_s = session_get()->speed_mm_s;
  g_ax[0].accel_mm_s2 = session_get()->accel_mm_s2;
  planner_request_move_by(0, mm);
  return true;
}

bool motion_jog(int dir, int axis_mask) {
  if (!g_enabled) {
    return false;
  }
  int sign = dir >= 0 ? 1 : -1;
  if (!config_axis2_enabled() || axis_mask == 0) {
    /* 1-axis mode or mask=both: jog all active axes */
    for (int axis = 0; axis < axis_count(); ++axis) {
      if (planner_hard_limit_blocks_sign(axis, sign)) {
        return false;
      }
    }
    joy_clear();
    coord_clear();
    apply_session_cruise_accel();
    for (int axis = 0; axis < axis_count(); ++axis) {
      planner_request_jog(axis, dir);
    }
    return true;
  }
  if (axis_mask == 1) {
    if (planner_hard_limit_blocks_sign(0, sign)) {
      return false;
    }
    joy_clear();
    coord_clear();
    apply_session_cruise_accel();
    planner_request_jog(0, dir);
    return true;
  }
  if (axis_mask == 2) {
    if (planner_hard_limit_blocks_sign(1, sign)) {
      return false;
    }
    joy_clear();
    coord_clear();
    apply_session_cruise_accel();
    planner_request_jog(1, dir);
    return true;
  }
  return false;
}

bool motion_joy(float pct0, float pct1_or_nan) {
  if (!g_enabled) {
    return false;
  }
  coord_clear();
  g_joy_active = true;
  if (isnan(pct0) || fabsf(pct0) < 1e-3f) {
    pct0 = 0.0f;
  }
  g_joy_pct[0] = pct0;
  planner_request_joy(0, signed_cruise_from_pct(0, pct0));
  if (!config_axis2_enabled()) {
    g_joy_pct[1] = 0.0f;
    return true;
  }
  float pct1 = (isnan(pct1_or_nan) || fabsf(pct1_or_nan) < 1e-3f) ? 0.0f : pct1_or_nan;
  g_joy_pct[1] = pct1;
  planner_request_joy(1, signed_cruise_from_pct(1, pct1));
  return true;
}

void motion_end_joy(void) { joy_clear(); }

bool motion_stop(void) {
  joy_clear();
  planner_request_stop();
  return true;
}

bool motion_halt(void) {
  planner_request_halt();
  return true;
}

bool motion_home(int axis_1based) {
  if (!g_enabled) {
    return false;
  }
  int axis = axis_1based - 1;
  if (axis < 0 || axis >= axis_count()) {
    return false;
  }
  if (axis_hw_home_mode(axis) == 0) {
    joy_clear();
    return true;
  }
  if (!home_cfg_ok(axis)) {
    return false;
  }
  joy_clear();
  planner_request_home(axis);
  return true;
}

bool motion_soft_reset(void) {
  planner_soft_reset();
  return true;
}

bool motion_set_speed(float mm_s) {
  session_get()->speed_mm_s = mm_s;
  if (g_joy_active) {
    apply_joy_cruise_accel();
  } else {
    apply_session_cruise_accel();
  }
  return true;
}

bool motion_set_accel(float mm_s2) {
  session_get()->accel_mm_s2 = mm_s2;
  if (g_joy_active) {
    apply_joy_cruise_accel();
  } else {
    apply_session_cruise_accel();
  }
  return true;
}

bool motion_set_max_speed(float mm_s) {
  config_get()->max_speed_mm_s = mm_s;
  return true;
}

bool motion_set_window_left(bool set0, float mm0, bool set1, float mm1) {
  return session_set_window_left(set0, mm0, set1, mm1);
}

bool motion_set_window_right(bool set0, float mm0, bool set1, float mm1) {
  return session_set_window_right(set0, mm0, set1, mm1);
}

void motion_reset_window_left(void) { session_reset_left(); }

void motion_reset_window_right(void) { session_reset_right(); }

void motion_get_status(McStatus *out) {
  if (motion_path_is_active()) {
    motion_path_get_status(out);
    return;
  }
  planner_get_status(out);
}

bool motion_is_busy(void) { return planner_is_busy(); }

void motion_stub_tick_ms(unsigned ms) { (void)ms; }

#endif /* !HOST_TEST */
