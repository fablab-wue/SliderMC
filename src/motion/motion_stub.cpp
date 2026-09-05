#include "motion_api.h"
#include "config_store.h"
#include "motion_path.h"
#include "axis_hw.h"
#include "pins.h"

#include <math.h>

/*
 * Lightweight motion stub for protocol bring-up and host tests.
 * Interpolates position over virtual travel time (A → M → B → I) so WP/WC/WnC
 * and WM/WH can complete.
 */

static McStatus g_st;
static float g_pending_target;
static float g_pending_target_2;
static bool g_pending;
static bool g_pending_2;
static unsigned g_eta_ms;
static unsigned g_eta_ms_2;
static unsigned g_eta_total[2];
static float g_move_start[2];
static McState g_move_phase;
static float g_cruise[2];
static float g_accel[2];
static bool g_coord_active;
static float g_coord_ratio;
static bool g_joy_active;
static float g_joy_pct[2];

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
  if (g_st.moving || g_st.homing || g_pending || g_pending_2) {
    return;
  }
  coord_clear();
}

static void scale_cruise_accel(float v0, float a0, float ratio, float *v_out, float *a_out) {
  float v1 = v0 * ratio;
  float a1 = a0 * ratio;
  float vmax = config_get()->max_speed_mm_s_2;
  float amax = config_get()->max_accel_mm_s2_2;
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
  const McConfig *c = config_get();
  float mx = (axis == 1) ? c->max_speed_mm_s_2 : c->max_speed_mm_s;
  if (v > mx) {
    v = mx;
  }
  if (v < 0.0f) {
    v = 0.0f;
  }
  return v;
}

static float clamp_accel_axis(int axis, float a) {
  const McConfig *c = config_get();
  float mx = (axis == 1) ? c->max_accel_mm_s2_2 : c->max_accel_mm_s2;
  if (a > mx) {
    a = mx;
  }
  if (a < 0.001f) {
    a = 0.001f;
  }
  return a;
}

static void apply_cruise_accel(float v0, float a0) {
  g_cruise[0] = clamp_speed_axis(0, v0);
  g_accel[0] = clamp_accel_axis(0, a0);
  if (g_coord_active && config_axis2_enabled()) {
    scale_cruise_accel(v0, a0, g_coord_ratio, &g_cruise[1], &g_accel[1]);
  } else {
    g_cruise[1] = clamp_speed_axis(1, v0);
    g_accel[1] = clamp_accel_axis(1, a0);
  }
  /* Refresh live status speeds if a move is in progress. */
  if (g_pending) {
    g_st.vel_mm_s = (g_pending_target >= g_st.pos_mm) ? g_cruise[0] : -g_cruise[0];
    g_st.acc_mm_s2 = g_accel[0];
  }
  if (g_pending_2) {
    g_st.vel_mm_s_2 = (g_pending_target_2 >= g_st.pos_mm_2) ? g_cruise[1] : -g_cruise[1];
    g_st.acc_mm_s2_2 = g_accel[1];
  }
}

static void apply_session_cruise_accel(void) {
  apply_cruise_accel(session_get()->speed_mm_s, session_get()->accel_mm_s2);
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
  for (int axis = 0; axis < 2; ++axis) {
    g_accel[axis] = clamp_accel_axis(axis, session_get()->accel_mm_s2);
    float pct = g_joy_pct[axis];
    if (fabsf(pct) < 1e-3f) {
      continue;
    }
    float signed_v = signed_cruise_from_pct(axis, pct);
    g_cruise[axis] = fabsf(signed_v);
    if (axis == 0 && g_pending) {
      g_st.vel_mm_s = (signed_v >= 0.0f) ? g_cruise[0] : -g_cruise[0];
      g_st.acc_mm_s2 = g_accel[0];
    }
    if (axis == 1 && g_pending_2) {
      g_st.vel_mm_s_2 = (signed_v >= 0.0f) ? g_cruise[1] : -g_cruise[1];
      g_st.acc_mm_s2_2 = g_accel[1];
    }
  }
}

void motion_init(void) {
  g_st.state = MC_STATE_DISABLED;
  g_st.enabled = false;
  g_st.moving = false;
  g_st.homing = false;
  g_st.at_soft_limit = false;
  g_st.hard_limit = false;
  g_st.drv_error = false;
  g_st.has_target = false;
  g_st.pos_mm = 0.0f;
  g_st.pos_mm_2 = 0.0f;
  g_st.target_mm = 0.0f;
  g_st.target_mm_2 = 0.0f;
  g_st.vel_mm_s = 0.0f;
  g_st.vel_mm_s_2 = 0.0f;
  g_st.acc_mm_s2 = 0.0f;
  g_st.acc_mm_s2_2 = 0.0f;
  g_pending = false;
  g_pending_2 = false;
  g_eta_ms = 0;
  g_eta_ms_2 = 0;
  g_eta_total[0] = g_eta_total[1] = 0;
  g_move_start[0] = g_move_start[1] = 0.0f;
  g_move_phase = MC_STATE_IDLE;
  coord_clear();
  joy_clear();
  apply_session_cruise_accel();
  motion_path_init();
}

static void refresh_state(void) {
  if (g_st.drv_error) {
    g_st.state = MC_STATE_ERROR;
  } else if (!g_st.enabled) {
    g_st.state = MC_STATE_DISABLED;
  } else if (g_st.homing) {
    g_st.state = MC_STATE_HOMING;
  } else if (g_st.moving) {
    g_st.state = g_move_phase;
  } else {
    g_st.state = MC_STATE_IDLE;
  }
}

static bool soft_ok(float dest, int axis) {
  float mn = axis_hw_window_min(axis);
  float mx = axis_hw_window_max(axis);
  if (!isnan(mn) && dest < mn) {
    g_st.at_soft_limit = true;
    return false;
  }
  if (!isnan(mx) && dest > mx) {
    g_st.at_soft_limit = true;
    return false;
  }
  g_st.at_soft_limit = false;
  return true;
}

static void start_move_axis(int axis, float dest) {
  float pos = (axis == 1) ? g_st.pos_mm_2 : g_st.pos_mm;
  float dist = fabsf(dest - pos);
  float v = g_cruise[axis];
  if (v < 0.001f) {
    v = 0.001f;
  }
  unsigned ms = (unsigned)(dist / v * 1000.0f);
  if (ms < 20) {
    ms = 20;
  }
  g_move_start[axis] = pos;
  g_eta_total[axis] = ms;
  g_move_phase = MC_STATE_ACCELERATING;
  if (axis == 1) {
    g_pending_target_2 = dest;
    g_pending_2 = true;
    g_eta_ms_2 = ms;
    g_st.target_mm_2 = dest;
    g_st.vel_mm_s_2 = (dest >= g_st.pos_mm_2) ? v : -v;
    g_st.acc_mm_s2_2 = g_accel[1];
  } else {
    g_pending_target = dest;
    g_pending = true;
    g_eta_ms = ms;
    g_st.target_mm = dest;
    g_st.vel_mm_s = (dest >= g_st.pos_mm) ? v : -v;
    g_st.acc_mm_s2 = g_accel[0];
    g_st.has_target = true;
  }
  g_st.moving = true;
  refresh_state();
}

bool motion_enable(bool on) {
  g_st.enabled = on;
  if (!on) {
    g_st.moving = false;
    g_st.homing = false;
    g_pending = false;
    g_pending_2 = false;
    g_st.vel_mm_s = 0.0f;
    g_st.vel_mm_s_2 = 0.0f;
    g_st.acc_mm_s2 = 0.0f;
    g_st.acc_mm_s2_2 = 0.0f;
    g_st.has_target = false;
    joy_clear();
    coord_clear();
  }
  refresh_state();
  return true;
}

bool motion_move_to(float mm) {
  if (!g_st.enabled || g_st.drv_error) {
    return false;
  }
  if (!soft_ok(mm, 0)) {
    return false;
  }
  joy_clear();
  coord_clear();
  apply_session_cruise_accel();
  start_move_axis(0, mm);
  return true;
}

bool motion_move_to2(float mm1_or_nan, float mm2_or_nan) {
  if (!g_st.enabled || g_st.drv_error) {
    return false;
  }
  bool do0 = !isnan(mm1_or_nan);
  bool do1 = config_axis2_enabled() && !isnan(mm2_or_nan);
  if (do0 && !soft_ok(mm1_or_nan, 0)) {
    return false;
  }
  if (do1 && !soft_ok(mm2_or_nan, 1)) {
    return false;
  }
  float d0 = do0 ? (mm1_or_nan - g_st.pos_mm) : 0.0f;
  float d1 = do1 ? (mm2_or_nan - g_st.pos_mm_2) : 0.0f;
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
  joy_clear();
  if (do0 && do1) {
    g_coord_active = true;
    g_coord_ratio = fabsf(d1) / fabsf(d0);
    apply_session_cruise_accel();
  } else {
    coord_clear();
    apply_session_cruise_accel();
  }
  if (do0) {
    start_move_axis(0, mm1_or_nan);
  }
  if (do1) {
    start_move_axis(1, mm2_or_nan);
  }
  return true;
}

bool motion_move_by(float mm) { return motion_move_to(g_st.pos_mm + mm); }

bool motion_jog(int dir, int axis_mask) {
  if (!g_st.enabled || g_st.drv_error) {
    return false;
  }
  const McConfig *c = config_get();
  float delta = (dir >= 0 ? 1.0f : -1.0f) * c->max_speed_mm_s * 10.0f;
  bool ax2 = config_axis2_enabled();
  if (!ax2 || axis_mask == 0 || axis_mask == 1) {
    float dest = g_st.pos_mm + delta;
    if (!isnan(axis_hw_window_max(0)) && dest > axis_hw_window_max(0)) {
      dest = axis_hw_window_max(0);
    }
    if (!isnan(axis_hw_window_min(0)) && dest < axis_hw_window_min(0)) {
      dest = axis_hw_window_min(0);
    }
    if (!soft_ok(dest, 0)) {
      return false;
    }
    joy_clear();
    coord_clear();
    apply_session_cruise_accel();
    start_move_axis(0, dest);
  }
  if (ax2 && (axis_mask == 0 || axis_mask == 2)) {
    float dest = g_st.pos_mm_2 + delta;
    if (!isnan(axis_hw_window_max(1)) && dest > axis_hw_window_max(1)) {
      dest = axis_hw_window_max(1);
    }
    if (!isnan(axis_hw_window_min(1)) && dest < axis_hw_window_min(1)) {
      dest = axis_hw_window_min(1);
    }
    if (!ax2 || axis_mask == 2) {
      joy_clear();
      coord_clear();
      apply_session_cruise_accel();
    }
    start_move_axis(1, dest);
  }
  return true;
}

static float joy_dest_mm(int axis, int sign) {
  float mn = axis_hw_window_min(axis);
  float mx = axis_hw_window_max(axis);
  if (sign > 0) {
    return isnan(mx) ? 1e9f : mx;
  }
  return isnan(mn) ? -1e9f : mn;
}

static void stop_axis_stub(int axis) {
  if (axis == 0) {
    g_pending = false;
    g_st.vel_mm_s = 0.0f;
    g_st.acc_mm_s2 = 0.0f;
  } else {
    g_pending_2 = false;
    g_st.vel_mm_s_2 = 0.0f;
    g_st.acc_mm_s2_2 = 0.0f;
  }
  if (!g_pending && !g_pending_2) {
    g_st.moving = false;
    g_st.has_target = false;
  }
  refresh_state();
}

static void request_joy_axis(int axis, float signed_v) {
  g_accel[axis] = clamp_accel_axis(axis, session_get()->accel_mm_s2);
  if (fabsf(signed_v) < 1e-4f) {
    stop_axis_stub(axis);
    return;
  }
  int sign = signed_v > 0.0f ? 1 : -1;
  float dest = joy_dest_mm(axis, sign);
  float pos = (axis == 1) ? g_st.pos_mm_2 : g_st.pos_mm;
  if ((sign > 0 && dest <= pos + 1e-6f) || (sign < 0 && dest >= pos - 1e-6f)) {
    stop_axis_stub(axis);
    return;
  }
  g_cruise[axis] = fabsf(signed_v);
  start_move_axis(axis, dest);
}

bool motion_joy(float pct0, float pct1_or_nan) {
  if (!g_st.enabled || g_st.drv_error) {
    return false;
  }
  coord_clear();
  g_joy_active = true;
  if (isnan(pct0) || fabsf(pct0) < 1e-3f) {
    pct0 = 0.0f;
  }
  g_joy_pct[0] = pct0;
  request_joy_axis(0, signed_cruise_from_pct(0, pct0));
  if (!config_axis2_enabled()) {
    g_joy_pct[1] = 0.0f;
    return true;
  }
  float pct1 = (isnan(pct1_or_nan) || fabsf(pct1_or_nan) < 1e-3f) ? 0.0f : pct1_or_nan;
  g_joy_pct[1] = pct1;
  request_joy_axis(1, signed_cruise_from_pct(1, pct1));
  return true;
}

void motion_end_joy(void) { joy_clear(); }

bool motion_stop(void) {
  joy_clear();
  if (g_pending) {
    g_st.pos_mm = g_pending_target;
  }
  if (g_pending_2) {
    g_st.pos_mm_2 = g_pending_target_2;
  }
  g_pending = false;
  g_pending_2 = false;
  g_st.moving = false;
  g_st.homing = false;
  g_st.has_target = false;
  g_st.vel_mm_s = 0.0f;
  g_st.vel_mm_s_2 = 0.0f;
  g_st.acc_mm_s2 = 0.0f;
  g_st.acc_mm_s2_2 = 0.0f;
  refresh_state();
  coord_clear_if_idle();
  return true;
}

bool motion_halt(void) {
  g_pending = false;
  g_pending_2 = false;
  g_st.moving = false;
  g_st.homing = false;
  g_st.has_target = false;
  g_st.vel_mm_s = 0.0f;
  g_st.vel_mm_s_2 = 0.0f;
  g_st.acc_mm_s2 = 0.0f;
  g_st.acc_mm_s2_2 = 0.0f;
  g_st.enabled = false;
  joy_clear();
  coord_clear();
  refresh_state();
  return true;
}

bool motion_home(int axis_1based) {
  if (!g_st.enabled) {
    return false;
  }
  int axis = axis_1based - 1;
  if (axis < 0 || axis > 1 || (axis == 1 && !config_axis2_enabled())) {
    return false;
  }
  const McConfig *c = config_get();
  int hm = (axis == 1) ? c->home_mode_2 : c->home_mode;
  if (hm == 0) {
    joy_clear();
    return true;
  }
  bool cfg_ok = false;
  if (hm == 1) {
    cfg_ok = ((axis == 1) ? c->sw_limit_l_use_2 : c->sw_limit_l_use) != 0;
  } else if (hm == 2) {
    cfg_ok = ((axis == 1) ? c->sw_limit_r_use_2 : c->sw_limit_r_use) != 0;
  } else if (hm == 3 || hm == 4) {
    cfg_ok = true;
  }
  if (!cfg_ok) {
    return false;
  }
  joy_clear();
  float mn = (axis == 1) ? c->slider_min_mm_2 : c->slider_min_mm;
  float mx = (axis == 1) ? c->slider_max_mm_2 : c->slider_max_mm;
  float target;
  if (hm == 1 || hm == 3) {
    target = isnan(mn) ? 0.0f : mn;
  } else {
    target = isnan(mx) ? 0.0f : mx;
  }
  float v = (axis == 1) ? c->home_speed_mm_s_2 : c->home_speed_mm_s;
  if (v < 0.001f) {
    v = 0.001f;
  }
  float pos = (axis == 1) ? g_st.pos_mm_2 : g_st.pos_mm;
  float out = (axis == 1) ? c->home_move_out_mm_2 : c->home_move_out_mm;
  float dist = fabsf(target - pos) + out;
  unsigned ms = (unsigned)(dist / v * 1000.0f);
  if (ms < 50) {
    ms = 50;
  }
  g_st.homing = true;
  g_st.moving = true;
  g_st.has_target = true;
  if (axis == 0) {
    g_st.target_mm = target;
    g_pending = true;
    g_pending_target = target;
    g_eta_ms = ms;
    g_eta_total[0] = ms;
    g_move_start[0] = g_st.pos_mm;
    g_st.vel_mm_s = (target >= g_st.pos_mm) ? v : -v;
    g_st.acc_mm_s2 = c->home_accel_mm_s2;
  } else {
    g_pending_2 = true;
    g_pending_target_2 = target;
    g_eta_ms_2 = ms;
    g_eta_total[1] = ms;
    g_move_start[1] = g_st.pos_mm_2;
  }
  refresh_state();
  return true;
}

bool motion_set_position(float mm0_or_nan, float mm1_or_nan) {
  if (g_st.moving || g_st.homing) {
    return false;
  }
  if (!isnan(mm0_or_nan)) {
    g_st.pos_mm = mm0_or_nan;
    g_st.target_mm = mm0_or_nan;
  }
  if (!isnan(mm1_or_nan) && config_axis2_enabled()) {
    g_st.pos_mm_2 = mm1_or_nan;
    g_st.target_mm_2 = mm1_or_nan;
  }
  refresh_state();
  return true;
}

bool motion_soft_reset(void) {
  g_st.drv_error = false;
  g_pending = false;
  g_pending_2 = false;
  g_st.moving = false;
  g_st.homing = false;
  g_st.has_target = false;
  g_st.vel_mm_s = 0.0f;
  g_st.vel_mm_s_2 = 0.0f;
  g_st.acc_mm_s2 = 0.0f;
  g_st.acc_mm_s2_2 = 0.0f;
  joy_clear();
  coord_clear();
  refresh_state();
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
  if (out) {
    *out = g_st;
    if (!config_axis2_enabled()) {
      out->pos_mm_2 = 0.0f;
      out->target_mm_2 = 0.0f;
      out->vel_mm_s_2 = 0.0f;
      out->acc_mm_s2_2 = 0.0f;
    }
  }
}

bool motion_is_busy(void) { return g_st.moving || g_st.homing; }

static McState phase_from_remain(unsigned remain, unsigned total) {
  if (total == 0) {
    return MC_STATE_MOVING;
  }
  float prog = 1.0f - (float)remain / (float)total;
  if (prog < 0.20f) {
    return MC_STATE_ACCELERATING;
  }
  if (prog > 0.80f) {
    return MC_STATE_DECELERATING;
  }
  return MC_STATE_MOVING;
}

static void stub_advance_axis(int axis, unsigned ms) {
  bool *pending = (axis == 1) ? &g_pending_2 : &g_pending;
  unsigned *eta = (axis == 1) ? &g_eta_ms_2 : &g_eta_ms;
  float *pos = (axis == 1) ? &g_st.pos_mm_2 : &g_st.pos_mm;
  float target = (axis == 1) ? g_pending_target_2 : g_pending_target;
  if (!*pending) {
    return;
  }
  if (ms >= *eta) {
    *eta = 0;
    *pos = target;
    *pending = false;
    return;
  }
  *eta -= ms;
  unsigned total = g_eta_total[axis];
  float prog = (total > 0) ? (1.0f - (float)*eta / (float)total) : 1.0f;
  if (prog < 0.0f) {
    prog = 0.0f;
  }
  if (prog > 1.0f) {
    prog = 1.0f;
  }
  *pos = g_move_start[axis] + (target - g_move_start[axis]) * prog;
}

void motion_stub_tick_ms(unsigned ms) {
  stub_advance_axis(0, ms);
  stub_advance_axis(1, ms);
  if (!g_pending && !g_pending_2) {
    g_st.moving = false;
    g_st.homing = false;
    g_st.has_target = false;
    g_st.vel_mm_s = 0.0f;
    g_st.vel_mm_s_2 = 0.0f;
    g_st.acc_mm_s2 = 0.0f;
    g_st.acc_mm_s2_2 = 0.0f;
    g_move_phase = MC_STATE_IDLE;
    refresh_state();
    coord_clear_if_idle();
    return;
  }
  McState p0 = g_pending ? phase_from_remain(g_eta_ms, g_eta_total[0]) : MC_STATE_IDLE;
  McState p1 = g_pending_2 ? phase_from_remain(g_eta_ms_2, g_eta_total[1]) : MC_STATE_IDLE;
  if (p0 == MC_STATE_DECELERATING || p1 == MC_STATE_DECELERATING) {
    g_move_phase = MC_STATE_DECELERATING;
  } else if (p0 == MC_STATE_ACCELERATING || p1 == MC_STATE_ACCELERATING) {
    g_move_phase = MC_STATE_ACCELERATING;
  } else {
    g_move_phase = MC_STATE_MOVING;
  }
  refresh_state();
}

#ifdef HOST_TEST
float motion_host_axis_cruise(int axis) {
  if (axis < 0 || axis > 1) {
    return 0.0f;
  }
  return g_cruise[axis];
}

float motion_host_axis_accel(int axis) {
  if (axis < 0 || axis > 1) {
    return 0.0f;
  }
  return g_accel[axis];
}
#endif
