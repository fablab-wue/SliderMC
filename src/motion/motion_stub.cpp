#include "motion_api.h"
#include "config_store.h"
#include "motion_path.h"
#include "axis_hw.h"
#include "pins.h"
#include "protocol.h"

#include <math.h>

/*
 * Lightweight motion stub for protocol bring-up and host tests.
 * Interpolates position over virtual travel time (A → M → B → I) so WP/WC/WnC
 * and WM/WH can complete.
 */

static McStatus g_st;
static float g_pending_target[3];
static bool g_pending[3];
static unsigned g_eta_ms[3];
static unsigned g_eta_total[3];
static float g_move_start[3];
static McState g_move_phase;
static float g_cruise[3];
static float g_accel[3];
static bool g_coord_active;
static int g_coord_master;
static float g_coord_ratio[3];
static bool g_joy_active;
static float g_joy_pct[3];

static int stub_naxes(void) { return config_axis_count(); }

static void coord_clear(void) {
  g_coord_active = false;
  g_coord_master = 0;
  g_coord_ratio[0] = g_coord_ratio[1] = g_coord_ratio[2] = 1.0f;
}

static void joy_clear(void) {
  g_joy_active = false;
  g_joy_pct[0] = g_joy_pct[1] = g_joy_pct[2] = 0.0f;
}

static bool any_pending(void) { return g_pending[0] || g_pending[1] || g_pending[2]; }

static void coord_clear_if_idle(void) {
  if (!g_coord_active) {
    return;
  }
  if (g_st.moving || g_st.homing || any_pending()) {
    return;
  }
  coord_clear();
}

static void scale_cruise_accel(int axis, float v0, float a0, float ratio, float *v_out,
                               float *a_out) {
  float v1 = v0 * ratio;
  float a1 = a0 * ratio;
  float vmax = axis_hw_max_speed(axis);
  float amax = axis_hw_max_accel(axis);
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

static float *st_pos(int axis) {
  if (axis == 2) {
    return &g_st.pos_mm_3;
  }
  if (axis == 1) {
    return &g_st.pos_mm_2;
  }
  return &g_st.pos_mm;
}

static float *st_tgt(int axis) {
  if (axis == 2) {
    return &g_st.target_mm_3;
  }
  if (axis == 1) {
    return &g_st.target_mm_2;
  }
  return &g_st.target_mm;
}

static float *st_vel(int axis) {
  if (axis == 2) {
    return &g_st.vel_mm_s_3;
  }
  if (axis == 1) {
    return &g_st.vel_mm_s_2;
  }
  return &g_st.vel_mm_s;
}

static float *st_acc(int axis) {
  if (axis == 2) {
    return &g_st.acc_mm_s2_3;
  }
  if (axis == 1) {
    return &g_st.acc_mm_s2_2;
  }
  return &g_st.acc_mm_s2;
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

static void apply_cruise_accel(float v0, float a0) {
  int n = stub_naxes();
  int master = g_coord_active ? g_coord_master : 0;
  if (master < 0 || master >= n) {
    master = 0;
  }
  if (g_coord_active) {
    g_cruise[master] = clamp_speed_axis(master, v0);
    g_accel[master] = clamp_accel_axis(master, a0);
    for (int axis = 0; axis < n; ++axis) {
      if (axis == master) {
        continue;
      }
      scale_cruise_accel(axis, v0, a0, g_coord_ratio[axis], &g_cruise[axis], &g_accel[axis]);
    }
  } else {
    for (int axis = 0; axis < 3; ++axis) {
      g_cruise[axis] = clamp_speed_axis(axis, v0);
      g_accel[axis] = clamp_accel_axis(axis, a0);
    }
  }
  for (int axis = 0; axis < n; ++axis) {
    if (g_pending[axis]) {
      *st_vel(axis) = (g_pending_target[axis] >= *st_pos(axis)) ? g_cruise[axis] : -g_cruise[axis];
      *st_acc(axis) = g_accel[axis];
    }
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
  for (int axis = 0; axis < stub_naxes(); ++axis) {
    g_accel[axis] = clamp_accel_axis(axis, session_get()->accel_mm_s2);
    float pct = g_joy_pct[axis];
    if (fabsf(pct) < 1e-3f) {
      continue;
    }
    float signed_v = signed_cruise_from_pct(axis, pct);
    g_cruise[axis] = fabsf(signed_v);
    if (g_pending[axis]) {
      *st_vel(axis) = (signed_v >= 0.0f) ? g_cruise[axis] : -g_cruise[axis];
      *st_acc(axis) = g_accel[axis];
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
  g_st.pos_mm_3 = 0.0f;
  g_st.target_mm = 0.0f;
  g_st.target_mm_2 = 0.0f;
  g_st.target_mm_3 = 0.0f;
  g_st.vel_mm_s = 0.0f;
  g_st.vel_mm_s_2 = 0.0f;
  g_st.vel_mm_s_3 = 0.0f;
  g_st.acc_mm_s2 = 0.0f;
  g_st.acc_mm_s2_2 = 0.0f;
  g_st.acc_mm_s2_3 = 0.0f;
  for (int i = 0; i < 3; ++i) {
    g_pending[i] = false;
    g_eta_ms[i] = 0;
    g_eta_total[i] = 0;
    g_move_start[i] = 0.0f;
    g_pending_target[i] = 0.0f;
  }
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
  float pos = *st_pos(axis);
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
  g_pending_target[axis] = dest;
  g_pending[axis] = true;
  g_eta_ms[axis] = ms;
  *st_tgt(axis) = dest;
  *st_vel(axis) = (dest >= pos) ? v : -v;
  *st_acc(axis) = g_accel[axis];
  if (axis == 0) {
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
    g_pending[0] = g_pending[1] = g_pending[2] = false;
    g_st.vel_mm_s = 0.0f;
    g_st.vel_mm_s_2 = 0.0f;
    g_st.vel_mm_s_3 = 0.0f;
    g_st.acc_mm_s2 = 0.0f;
    g_st.acc_mm_s2_2 = 0.0f;
    g_st.acc_mm_s2_3 = 0.0f;
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

bool motion_move_to_n(float mm0_or_nan, float mm1_or_nan, float mm2_or_nan) {
  if (!g_st.enabled || g_st.drv_error) {
    return false;
  }
  float dest[3] = {mm0_or_nan, mm1_or_nan, mm2_or_nan};
  bool want[3] = {false, false, false};
  float dist[3] = {0.0f, 0.0f, 0.0f};
  int n = stub_naxes();
  int movers = 0;
  int master = -1;
  for (int axis = 0; axis < n; ++axis) {
    if (isnan(dest[axis])) {
      continue;
    }
    if (!soft_ok(dest[axis], axis)) {
      return false;
    }
    dist[axis] = dest[axis] - *st_pos(axis);
    if (fabsf(dist[axis]) < 1e-6f) {
      continue;
    }
    want[axis] = true;
    if (master < 0) {
      master = axis;
    }
    ++movers;
  }
  if (movers == 0) {
    joy_clear();
    coord_clear();
    return true;
  }
  joy_clear();
  if (movers >= 2) {
    g_coord_active = true;
    g_coord_master = master;
    float dmaster = fabsf(dist[master]);
    if (dmaster < 1e-6f) {
      dmaster = 1e-6f;
    }
    for (int axis = 0; axis < n; ++axis) {
      g_coord_ratio[axis] = want[axis] ? (fabsf(dist[axis]) / dmaster) : 0.0f;
    }
    apply_session_cruise_accel();
  } else {
    coord_clear();
    apply_session_cruise_accel();
  }
  for (int axis = 0; axis < n; ++axis) {
    if (want[axis]) {
      start_move_axis(axis, dest[axis]);
    }
  }
  return true;
}

bool motion_move_to2(float mm1_or_nan, float mm2_or_nan) {
  return motion_move_to_n(mm1_or_nan, mm2_or_nan, NAN);
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
  if (config_axis3_enabled() && (axis_mask == 0 || axis_mask == 3)) {
    float dest = g_st.pos_mm_3 + delta;
    if (!isnan(axis_hw_window_max(2)) && dest > axis_hw_window_max(2)) {
      dest = axis_hw_window_max(2);
    }
    if (!isnan(axis_hw_window_min(2)) && dest < axis_hw_window_min(2)) {
      dest = axis_hw_window_min(2);
    }
    start_move_axis(2, dest);
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
  g_pending[axis] = false;
  *st_vel(axis) = 0.0f;
  *st_acc(axis) = 0.0f;
  if (!any_pending()) {
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
  float pos = *st_pos(axis);
  float mn = axis_hw_window_min(axis);
  float mx = axis_hw_window_max(axis);
  if (sign < 0 && !isnan(mn) && pos < mn) {
    dest = mn; /* outside left wall: snap inward */
    *st_pos(axis) = dest;
    stop_axis_stub(axis);
    return;
  }
  if (sign > 0 && !isnan(mx) && pos > mx) {
    dest = mx;
    *st_pos(axis) = dest;
    stop_axis_stub(axis);
    return;
  }
  if ((sign > 0 && dest <= pos + 1e-6f) || (sign < 0 && dest >= pos - 1e-6f)) {
    stop_axis_stub(axis);
    return;
  }
  g_cruise[axis] = fabsf(signed_v);
  start_move_axis(axis, dest);
}

bool motion_joy(float pct0, float pct1_or_nan, float pct2_or_nan) {
  if (!g_st.enabled || g_st.drv_error) {
    return false;
  }
  coord_clear();
  g_joy_active = true;
  float pct[3] = {pct0, pct1_or_nan, pct2_or_nan};
  int n = stub_naxes();
  for (int axis = 0; axis < 3; ++axis) {
    float p = pct[axis];
    if (axis >= n || isnan(p) || fabsf(p) < 1e-3f) {
      p = 0.0f;
    }
    g_joy_pct[axis] = p;
    if (axis < n) {
      request_joy_axis(axis, signed_cruise_from_pct(axis, p));
    }
  }
  return true;
}

void motion_end_joy(void) { joy_clear(); }

bool motion_stop(void) {
  joy_clear();
  for (int axis = 0; axis < 3; ++axis) {
    if (g_pending[axis]) {
      *st_pos(axis) = g_pending_target[axis];
    }
    g_pending[axis] = false;
    *st_vel(axis) = 0.0f;
    *st_acc(axis) = 0.0f;
  }
  g_st.moving = false;
  g_st.homing = false;
  g_st.has_target = false;
  refresh_state();
  coord_clear_if_idle();
  return true;
}

bool motion_halt(void) {
  g_pending[0] = g_pending[1] = g_pending[2] = false;
  g_st.moving = false;
  g_st.homing = false;
  g_st.has_target = false;
  g_st.vel_mm_s = 0.0f;
  g_st.vel_mm_s_2 = 0.0f;
  g_st.vel_mm_s_3 = 0.0f;
  g_st.acc_mm_s2 = 0.0f;
  g_st.acc_mm_s2_2 = 0.0f;
  g_st.acc_mm_s2_3 = 0.0f;
  joy_clear();
  coord_clear();
  protocol_cancel_waits_and_chain();
  refresh_state();
  return true;
}

bool motion_home(int axis_1based) {
  if (!g_st.enabled) {
    return false;
  }
  int axis = axis_1based - 1;
  if (axis < 0 || axis >= stub_naxes()) {
    return false;
  }
  int hm = axis_hw_home_mode(axis);
  if (hm == 0) {
    joy_clear();
    return true;
  }
  bool cfg_ok = false;
  if (hm == 1) {
    cfg_ok = axis_hw_limit_l_use(axis) != 0;
  } else if (hm == 2) {
    cfg_ok = axis_hw_limit_r_use(axis) != 0;
  } else if (hm == 3 || hm == 4) {
    cfg_ok = true;
  }
  if (!cfg_ok) {
    return false;
  }
  joy_clear();
  float mn = axis_hw_slider_min(axis);
  float mx = axis_hw_slider_max(axis);
  float target;
  if (hm == 1 || hm == 3) {
    target = isnan(mn) ? 0.0f : mn;
  } else {
    target = isnan(mx) ? 0.0f : mx;
  }
  float v = axis_hw_home_speed(axis);
  if (v < 0.001f) {
    v = 0.001f;
  }
  float pos = *st_pos(axis);
  float out = axis_hw_home_move_out(axis);
  float dist = fabsf(target - pos) + out;
  unsigned ms = (unsigned)(dist / v * 1000.0f);
  if (ms < 50) {
    ms = 50;
  }
  g_st.homing = true;
  g_st.moving = true;
  g_st.has_target = true;
  *st_tgt(axis) = target;
  g_pending[axis] = true;
  g_pending_target[axis] = target;
  g_eta_ms[axis] = ms;
  g_eta_total[axis] = ms;
  g_move_start[axis] = pos;
  *st_vel(axis) = (target >= pos) ? v : -v;
  *st_acc(axis) = axis_hw_home_accel(axis);
  refresh_state();
  return true;
}

bool motion_set_position(float mm0_or_nan, float mm1_or_nan, float mm2_or_nan) {
  if (g_st.moving || g_st.homing) {
    return false;
  }
  float mm[3] = {mm0_or_nan, mm1_or_nan, mm2_or_nan};
  int n = stub_naxes();
  for (int axis = 0; axis < n; ++axis) {
    if (!isnan(mm[axis])) {
      *st_pos(axis) = mm[axis];
      *st_tgt(axis) = mm[axis];
    }
  }
  refresh_state();
  return true;
}

bool motion_soft_reset(void) {
  g_st.drv_error = false;
  g_pending[0] = g_pending[1] = g_pending[2] = false;
  g_st.moving = false;
  g_st.homing = false;
  g_st.has_target = false;
  g_st.vel_mm_s = 0.0f;
  g_st.vel_mm_s_2 = 0.0f;
  g_st.vel_mm_s_3 = 0.0f;
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

bool motion_set_window_left(bool set0, float mm0, bool set1, float mm1, bool set2, float mm2) {
  return session_set_window_left(set0, mm0, set1, mm1, set2, mm2);
}

bool motion_set_window_right(bool set0, float mm0, bool set1, float mm1, bool set2, float mm2) {
  return session_set_window_right(set0, mm0, set1, mm1, set2, mm2);
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
    if (!config_axis3_enabled()) {
      out->pos_mm_3 = 0.0f;
      out->target_mm_3 = 0.0f;
      out->vel_mm_s_3 = 0.0f;
      out->acc_mm_s2_3 = 0.0f;
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
  if (!g_pending[axis]) {
    return;
  }
  if (ms >= g_eta_ms[axis]) {
    g_eta_ms[axis] = 0;
    *st_pos(axis) = g_pending_target[axis];
    g_pending[axis] = false;
    return;
  }
  g_eta_ms[axis] -= ms;
  unsigned total = g_eta_total[axis];
  float prog = (total > 0) ? (1.0f - (float)g_eta_ms[axis] / (float)total) : 1.0f;
  if (prog < 0.0f) {
    prog = 0.0f;
  }
  if (prog > 1.0f) {
    prog = 1.0f;
  }
  *st_pos(axis) = g_move_start[axis] + (g_pending_target[axis] - g_move_start[axis]) * prog;
}

void motion_stub_tick_ms(unsigned ms) {
  stub_advance_axis(0, ms);
  stub_advance_axis(1, ms);
  stub_advance_axis(2, ms);
  if (!any_pending()) {
    g_st.moving = false;
    g_st.homing = false;
    g_st.has_target = false;
    g_st.vel_mm_s = 0.0f;
    g_st.vel_mm_s_2 = 0.0f;
    g_st.vel_mm_s_3 = 0.0f;
    g_st.acc_mm_s2 = 0.0f;
    g_st.acc_mm_s2_2 = 0.0f;
    g_st.acc_mm_s2_3 = 0.0f;
    g_move_phase = MC_STATE_IDLE;
    refresh_state();
    coord_clear_if_idle();
    return;
  }
  McState phase = MC_STATE_IDLE;
  for (int axis = 0; axis < 3; ++axis) {
    if (!g_pending[axis]) {
      continue;
    }
    McState p = phase_from_remain(g_eta_ms[axis], g_eta_total[axis]);
    if (p == MC_STATE_DECELERATING) {
      phase = MC_STATE_DECELERATING;
    } else if (p == MC_STATE_ACCELERATING && phase != MC_STATE_DECELERATING) {
      phase = MC_STATE_ACCELERATING;
    } else if (phase == MC_STATE_IDLE) {
      phase = MC_STATE_MOVING;
    }
  }
  g_move_phase = phase;
  refresh_state();
}

#ifdef HOST_TEST
float motion_host_axis_cruise(int axis) {
  if (axis < 0 || axis > 2) {
    return 0.0f;
  }
  return g_cruise[axis];
}

float motion_host_axis_accel(int axis) {
  if (axis < 0 || axis > 2) {
    return 0.0f;
  }
  return g_accel[axis];
}
#endif
