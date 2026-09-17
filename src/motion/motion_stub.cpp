#include "motion_api.h"
#include "config_store.h"
#include "motion_path.h"
#include "axis_hw.h"
#include "pins.h"
#include "protocol.h"
#include "servo_pwm.h"

#include <math.h>
#include <string.h>

/*
 * Lightweight motion stub for protocol bring-up and host tests.
 * Interpolates position over virtual travel time (A → M → B → I) so WP/WC/WN
 * and WM/WH can complete.
 */

static McStatus g_st;
static float g_pending_target[MC_CH_MAX];
static bool g_pending[MC_CH_MAX];
static unsigned g_eta_ms[MC_CH_MAX];
static unsigned g_eta_total[MC_CH_MAX];
static float g_move_start[MC_CH_MAX];
static McState g_move_phase;
static float g_cruise[MC_CH_MAX];
static float g_accel[MC_CH_MAX];
static float g_decel[MC_CH_MAX];
static bool g_coord_active;
static int g_coord_master;
static float g_coord_ratio[MC_CH_MAX];
static bool g_joy_active;
static float g_joy_pct[MC_CH_MAX];
static int g_move_master = -1;

static int stub_nch(void) { return config_axis_count(); }
static int stub_nm(void) { return config_motor_count(); }

static bool ch_is_motor(int ch) { return ch >= 0 && ch < stub_nm(); }

static float ch_max_speed(int ch) {
  if (ch_is_motor(ch)) {
    return axis_hw_max_speed(ch);
  }
  int s = ch - stub_nm();
  if (s < 0 || s >= SERVO_MAX) {
    return 1.0f;
  }
  return config_get()->servo_max_speed[s];
}

static float ch_max_accel(int ch) {
  if (ch_is_motor(ch)) {
    return axis_hw_max_accel(ch);
  }
  int s = ch - stub_nm();
  if (s < 0 || s >= SERVO_MAX) {
    return 1.0f;
  }
  return config_get()->servo_max_accel[s];
}

static void coord_clear(void) {
  g_coord_active = false;
  g_coord_master = 0;
  g_move_master = -1;
  for (int i = 0; i < MC_CH_MAX; ++i) {
    g_coord_ratio[i] = 1.0f;
  }
}

static void joy_clear(void) {
  g_joy_active = false;
  for (int i = 0; i < MC_CH_MAX; ++i) {
    g_joy_pct[i] = 0.0f;
  }
}

static bool any_pending(void) {
  for (int i = 0; i < MC_CH_MAX; ++i) {
    if (g_pending[i]) {
      return true;
    }
  }
  return false;
}

static void coord_clear_if_idle(void) {
  if (!g_coord_active) {
    return;
  }
  if (g_st.moving || g_st.homing || any_pending()) {
    return;
  }
  coord_clear();
}

static void scale_cruise_accel(int ch, float v0, float a0, float d0, float ratio, float *v_out,
                               float *a_out, float *d_out) {
  float v1 = v0 * ratio;
  float a1 = a0 * ratio;
  float d1 = d0 * ratio;
  float vmax = ch_max_speed(ch);
  float amax = ch_max_accel(ch);
  if (v1 > vmax) {
    v1 = vmax;
  }
  if (a1 > amax) {
    a1 = amax;
  }
  if (d1 > amax) {
    d1 = amax;
  }
  if (v1 < 0.001f) {
    v1 = 0.001f;
  }
  if (a1 < 0.001f) {
    a1 = 0.001f;
  }
  if (d1 < 0.001f) {
    d1 = 0.001f;
  }
  *v_out = v1;
  *a_out = a1;
  *d_out = d1;
}

static float clamp_speed_ch(int ch, float v) {
  float mx = ch_max_speed(ch);
  if (v > mx) {
    v = mx;
  }
  if (v < 0.0f) {
    v = 0.0f;
  }
  return v;
}

static float clamp_accel_ch(int ch, float a) {
  float mx = ch_max_accel(ch);
  if (a > mx) {
    a = mx;
  }
  if (a < 0.001f) {
    a = 0.001f;
  }
  return a;
}

static void apply_cruise_accel(float v0, float a0, float d0) {
  int n = stub_nch();
  int master = g_coord_active ? g_coord_master : 0;
  if (master < 0 || master >= n) {
    master = 0;
  }
  if (g_coord_active) {
    g_cruise[master] = clamp_speed_ch(master, v0);
    g_accel[master] = clamp_accel_ch(master, a0);
    g_decel[master] = clamp_accel_ch(master, d0);
    for (int ch = 0; ch < n; ++ch) {
      if (ch == master) {
        continue;
      }
      scale_cruise_accel(ch, v0, a0, d0, g_coord_ratio[ch], &g_cruise[ch], &g_accel[ch],
                         &g_decel[ch]);
    }
  } else {
    for (int ch = 0; ch < MC_CH_MAX; ++ch) {
      g_cruise[ch] = clamp_speed_ch(ch, v0);
      g_accel[ch] = clamp_accel_ch(ch, a0);
      g_decel[ch] = clamp_accel_ch(ch, d0);
    }
  }
  for (int ch = 0; ch < n; ++ch) {
    if (g_pending[ch]) {
      g_st.vel[ch] = (g_pending_target[ch] >= g_st.pos[ch]) ? g_cruise[ch] : -g_cruise[ch];
      g_st.acc[ch] = g_accel[ch];
    }
  }
}

static void apply_session_cruise_accel(void) {
  apply_cruise_accel(session_get()->speed_mm_s, session_get()->accel_mm_s2,
                     session_get()->decel_mm_s2);
}

static float signed_cruise_from_pct(int ch, float pct) {
  if (fabsf(pct) < 1e-3f) {
    return 0.0f;
  }
  float v = fabsf(pct) * 0.01f * session_get()->speed_mm_s;
  v = clamp_speed_ch(ch, v);
  return (pct < 0.0f) ? -v : v;
}

static void apply_joy_cruise_accel(void) {
  for (int ch = 0; ch < stub_nch(); ++ch) {
    g_accel[ch] = clamp_accel_ch(ch, session_get()->accel_mm_s2);
    g_decel[ch] = clamp_accel_ch(ch, session_get()->decel_mm_s2);
    float pct = g_joy_pct[ch];
    if (fabsf(pct) < 1e-3f) {
      continue;
    }
    float signed_v = signed_cruise_from_pct(ch, pct);
    g_cruise[ch] = fabsf(signed_v);
    if (g_pending[ch]) {
      g_st.vel[ch] = (signed_v >= 0.0f) ? g_cruise[ch] : -g_cruise[ch];
      g_st.acc[ch] = g_accel[ch];
    }
  }
}

static float servo_boot(int s) {
  const McConfig *c = config_get();
  float mn = c->servo_min[s];
  float mx = c->servo_max[s];
  float p = 0.0f;
  if (!isnan(mn) && p < mn) {
    p = mn;
  }
  if (!isnan(mx) && p > mx) {
    p = mx;
  }
  return p;
}

void motion_init(void) {
  memset(&g_st, 0, sizeof(g_st));
  g_st.state = MC_STATE_DISABLED;
  g_st.enabled = false;
  int nm = stub_nm();
  int ns = config_servo_count();
  for (int s = 0; s < ns; ++s) {
    g_st.pos[nm + s] = servo_boot(s);
    g_st.target[nm + s] = g_st.pos[nm + s];
  }
  for (int i = 0; i < MC_CH_MAX; ++i) {
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
  servo_pwm_init();
  motion_path_init();
}

void motion_on_counts_changed(void) {
  int nm = stub_nm();
  int ns = config_servo_count();
  for (int s = 0; s < ns; ++s) {
    float p = servo_boot(s);
    g_st.pos[nm + s] = p;
    g_st.target[nm + s] = p;
    g_st.vel[nm + s] = 0.0f;
    g_st.acc[nm + s] = 0.0f;
    g_pending[nm + s] = false;
  }
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

static bool soft_ok(float dest, int ch) {
  float mn = session_effective_left(ch);
  float mx = session_effective_right(ch);
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

static void start_move_ch(int ch, float dest) {
  float pos = g_st.pos[ch];
  float dist = fabsf(dest - pos);
  float v = g_cruise[ch];
  if (v < 0.001f) {
    v = 0.001f;
  }
  unsigned ms = (unsigned)(dist / v * 1000.0f);
  if (ms < 20) {
    ms = 20;
  }
  g_move_start[ch] = pos;
  g_eta_total[ch] = ms;
  g_move_phase = MC_STATE_ACCELERATING;
  g_pending_target[ch] = dest;
  g_pending[ch] = true;
  g_eta_ms[ch] = ms;
  g_st.target[ch] = dest;
  g_st.vel[ch] = (dest >= pos) ? v : -v;
  g_st.acc[ch] = g_accel[ch];
  g_st.has_target = any_pending();
  g_st.moving = true;
  refresh_state();
}

bool motion_enable(bool on) {
  g_st.enabled = on;
  servo_pwm_set_enabled(on);
  if (!on) {
    g_st.moving = false;
    g_st.homing = false;
    for (int i = 0; i < MC_CH_MAX; ++i) {
      g_pending[i] = false;
      g_st.vel[i] = 0.0f;
      g_st.acc[i] = 0.0f;
    }
    g_st.has_target = false;
    joy_clear();
    coord_clear();
  }
  refresh_state();
  return true;
}

bool motion_move_to(float mm) {
  float dest[MC_CH_MAX];
  for (int i = 0; i < MC_CH_MAX; ++i) {
    dest[i] = NAN;
  }
  dest[0] = mm;
  return motion_move_to_n(dest);
}

bool motion_move_to_n(const float dest_in[MC_CH_MAX]) {
  if (!g_st.enabled || g_st.drv_error) {
    return false;
  }
  float dest[MC_CH_MAX];
  bool want[MC_CH_MAX];
  float dist[MC_CH_MAX];
  int n = stub_nch();
  int movers = 0;
  int master = -1;
  for (int ch = 0; ch < MC_CH_MAX; ++ch) {
    dest[ch] = dest_in[ch];
    want[ch] = false;
    dist[ch] = 0.0f;
  }
  for (int ch = 0; ch < n; ++ch) {
    if (isnan(dest[ch])) {
      continue;
    }
    if (!soft_ok(dest[ch], ch)) {
      return false;
    }
    dist[ch] = dest[ch] - g_st.pos[ch];
    if (fabsf(dist[ch]) < 1e-6f) {
      continue;
    }
    want[ch] = true;
    if (master < 0) {
      master = ch;
    }
    ++movers;
  }
  if (movers == 0) {
    joy_clear();
    coord_clear();
    return true;
  }
  joy_clear();
  g_move_master = master;
  if (movers >= 2) {
    g_coord_active = true;
    g_coord_master = master;
    float dmaster = fabsf(dist[master]);
    if (dmaster < 1e-6f) {
      dmaster = 1e-6f;
    }
    for (int ch = 0; ch < n; ++ch) {
      g_coord_ratio[ch] = want[ch] ? (fabsf(dist[ch]) / dmaster) : 0.0f;
    }
    apply_session_cruise_accel();
  } else {
    g_coord_active = false;
    g_coord_master = master;
    apply_session_cruise_accel();
    g_cruise[master] = clamp_speed_ch(master, session_get()->speed_mm_s);
    g_accel[master] = clamp_accel_ch(master, session_get()->accel_mm_s2);
    g_decel[master] = clamp_accel_ch(master, session_get()->decel_mm_s2);
  }
  for (int ch = 0; ch < n; ++ch) {
    if (want[ch]) {
      start_move_ch(ch, dest[ch]);
    }
  }
  return true;
}

bool motion_move_to2(float mm1_or_nan, float mm2_or_nan) {
  float dest[MC_CH_MAX];
  for (int i = 0; i < MC_CH_MAX; ++i) {
    dest[i] = NAN;
  }
  dest[0] = mm1_or_nan;
  dest[1] = mm2_or_nan;
  return motion_move_to_n(dest);
}

bool motion_move_by(float mm) {
  float dest[MC_CH_MAX];
  for (int i = 0; i < MC_CH_MAX; ++i) {
    dest[i] = NAN;
  }
  dest[0] = g_st.pos[0] + mm;
  return motion_move_to_n(dest);
}

static float joy_dest(int ch, int sign) {
  float mn = session_effective_left(ch);
  float mx = session_effective_right(ch);
  if (sign > 0) {
    return isnan(mx) ? 1e9f : mx;
  }
  return isnan(mn) ? -1e9f : mn;
}

static void stop_ch_stub(int ch) {
  g_pending[ch] = false;
  g_st.vel[ch] = 0.0f;
  g_st.acc[ch] = 0.0f;
  if (!any_pending()) {
    g_st.moving = false;
    g_st.has_target = false;
  }
  refresh_state();
}

static void request_joy_ch(int ch, float signed_v) {
  g_accel[ch] = clamp_accel_ch(ch, session_get()->accel_mm_s2);
  g_decel[ch] = clamp_accel_ch(ch, session_get()->decel_mm_s2);
  if (fabsf(signed_v) < 1e-4f) {
    stop_ch_stub(ch);
    return;
  }
  int sign = signed_v > 0.0f ? 1 : -1;
  float dest = joy_dest(ch, sign);
  float pos = g_st.pos[ch];
  float mn = session_effective_left(ch);
  float mx = session_effective_right(ch);
  if (sign < 0 && !isnan(mn) && pos < mn) {
    dest = mn;
    g_st.pos[ch] = dest;
    stop_ch_stub(ch);
    return;
  }
  if (sign > 0 && !isnan(mx) && pos > mx) {
    dest = mx;
    g_st.pos[ch] = dest;
    stop_ch_stub(ch);
    return;
  }
  if ((sign > 0 && dest <= pos + 1e-6f) || (sign < 0 && dest >= pos - 1e-6f)) {
    stop_ch_stub(ch);
    return;
  }
  g_cruise[ch] = fabsf(signed_v);
  start_move_ch(ch, dest);
}

bool motion_joy(const float pct_in[MC_CH_MAX]) {
  if (!g_st.enabled || g_st.drv_error) {
    return false;
  }
  coord_clear();
  g_joy_active = true;
  int n = stub_nch();
  for (int ch = 0; ch < MC_CH_MAX; ++ch) {
    float p = pct_in[ch];
    if (ch >= n || isnan(p) || fabsf(p) < 1e-3f) {
      p = 0.0f;
    }
    g_joy_pct[ch] = p;
    if (ch < n) {
      request_joy_ch(ch, signed_cruise_from_pct(ch, p));
    }
  }
  return true;
}

void motion_end_joy(void) { joy_clear(); }

bool motion_stop(void) {
  joy_clear();
  for (int ch = 0; ch < MC_CH_MAX; ++ch) {
    if (g_pending[ch]) {
      g_st.pos[ch] = g_pending_target[ch];
    }
    g_pending[ch] = false;
    g_st.vel[ch] = 0.0f;
    g_st.acc[ch] = 0.0f;
  }
  g_st.moving = false;
  g_st.homing = false;
  g_st.has_target = false;
  refresh_state();
  coord_clear_if_idle();
  return true;
}

bool motion_halt(void) {
  for (int i = 0; i < MC_CH_MAX; ++i) {
    g_pending[i] = false;
    g_st.vel[i] = 0.0f;
    g_st.acc[i] = 0.0f;
  }
  g_st.moving = false;
  g_st.homing = false;
  g_st.has_target = false;
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
  if (axis < 0 || axis >= stub_nm()) {
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
  float pos = g_st.pos[axis];
  float out = axis_hw_home_move_out(axis);
  float dist = fabsf(target - pos) + out;
  unsigned ms = (unsigned)(dist / v * 1000.0f);
  if (ms < 50) {
    ms = 50;
  }
  g_st.homing = true;
  g_st.moving = true;
  g_st.has_target = true;
  g_st.target[axis] = target;
  g_pending[axis] = true;
  g_pending_target[axis] = target;
  g_eta_ms[axis] = ms;
  g_eta_total[axis] = ms;
  g_move_start[axis] = pos;
  g_st.vel[axis] = (target >= pos) ? v : -v;
  g_st.acc[axis] = axis_hw_home_accel(axis);
  g_move_master = axis;
  refresh_state();
  return true;
}

bool motion_set_position(const float mm[MC_CH_MAX]) {
  if (g_st.moving || g_st.homing) {
    return false;
  }
  int n = stub_nch();
  for (int ch = 0; ch < n; ++ch) {
    if (!isnan(mm[ch])) {
      g_st.pos[ch] = mm[ch];
      g_st.target[ch] = mm[ch];
    }
  }
  refresh_state();
  return true;
}

bool motion_soft_reset(void) {
  g_st.drv_error = false;
  for (int i = 0; i < MC_CH_MAX; ++i) {
    g_pending[i] = false;
    g_st.vel[i] = 0.0f;
    g_st.acc[i] = 0.0f;
  }
  g_st.moving = false;
  g_st.homing = false;
  g_st.has_target = false;
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

bool motion_set_accel(float accel_mm_s2, float decel_mm_s2) {
  session_get()->accel_mm_s2 = accel_mm_s2;
  session_get()->decel_mm_s2 = decel_mm_s2;
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

bool motion_set_window_left(const bool set[MC_CH_MAX], const float mm[MC_CH_MAX]) {
  return session_set_window_left(set, mm);
}

bool motion_set_window_right(const bool set[MC_CH_MAX], const float mm[MC_CH_MAX]) {
  return session_set_window_right(set, mm);
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
  }
}

bool motion_is_busy(void) { return g_st.moving || g_st.homing; }

int motion_master_channel(void) {
  if (!g_st.moving && !g_st.homing) {
    return -1;
  }
  if (g_move_master >= 0) {
    return g_move_master;
  }
  return g_coord_active ? g_coord_master : 0;
}

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

static void stub_advance_ch(int ch, unsigned ms) {
  if (!g_pending[ch]) {
    return;
  }
  if (ms >= g_eta_ms[ch]) {
    g_eta_ms[ch] = 0;
    g_st.pos[ch] = g_pending_target[ch];
    g_pending[ch] = false;
    return;
  }
  g_eta_ms[ch] -= ms;
  unsigned total = g_eta_total[ch];
  float prog = (total > 0) ? (1.0f - (float)g_eta_ms[ch] / (float)total) : 1.0f;
  if (prog < 0.0f) {
    prog = 0.0f;
  }
  if (prog > 1.0f) {
    prog = 1.0f;
  }
  g_st.pos[ch] = g_move_start[ch] + (g_pending_target[ch] - g_move_start[ch]) * prog;
}

void motion_stub_tick_ms(unsigned ms) {
  for (int ch = 0; ch < MC_CH_MAX; ++ch) {
    stub_advance_ch(ch, ms);
  }
  if (!any_pending()) {
    g_st.moving = false;
    g_st.homing = false;
    g_st.has_target = false;
    for (int i = 0; i < MC_CH_MAX; ++i) {
      g_st.vel[i] = 0.0f;
      g_st.acc[i] = 0.0f;
    }
    g_move_phase = MC_STATE_IDLE;
    refresh_state();
    coord_clear_if_idle();
    return;
  }
  McState phase = MC_STATE_IDLE;
  for (int ch = 0; ch < MC_CH_MAX; ++ch) {
    if (!g_pending[ch]) {
      continue;
    }
    McState p = phase_from_remain(g_eta_ms[ch], g_eta_total[ch]);
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
  if (axis < 0 || axis >= MC_CH_MAX) {
    return 0.0f;
  }
  return g_cruise[axis];
}

float motion_host_axis_accel(int axis) {
  if (axis < 0 || axis >= MC_CH_MAX) {
    return 0.0f;
  }
  return g_accel[axis];
}

float motion_host_axis_decel(int axis) {
  if (axis < 0 || axis >= MC_CH_MAX) {
    return 0.0f;
  }
  return g_decel[axis];
}
#endif
