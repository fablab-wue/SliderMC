#include "motion_api.h"
#include "config_store.h"
#include "motion_path.h"

#include <math.h>

/*
 * Lightweight motion stub for protocol bring-up and host tests.
 * Instantly "arrives" after a short virtual travel time so WM/WH can complete.
 */

static McStatus g_st;
static float g_pending_target;
static bool g_pending;
static unsigned g_eta_ms;

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
  g_st.target_mm = 0.0f;
  g_st.vel_mm_s = 0.0f;
  g_st.acc_mm_s2 = 0.0f;
  g_pending = false;
  g_eta_ms = 0;
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
    g_st.state = MC_STATE_MOVING;
  } else {
    g_st.state = MC_STATE_IDLE;
  }
}

static bool soft_ok(float dest) {
  const McConfig *c = config_get();
  if (!isnan(c->slider_min_mm) && dest < c->slider_min_mm) {
    g_st.at_soft_limit = true;
    return false;
  }
  if (!isnan(c->slider_max_mm) && dest > c->slider_max_mm) {
    g_st.at_soft_limit = true;
    return false;
  }
  g_st.at_soft_limit = false;
  return true;
}

static void start_move(float dest) {
  const McSession *s = session_get();
  float dist = fabsf(dest - g_st.pos_mm);
  float v = s->speed_mm_s;
  if (v < 0.001f) {
    v = 0.001f;
  }
  unsigned ms = (unsigned)(dist / v * 1000.0f);
  if (ms < 20) {
    ms = 20;
  }
  g_pending_target = dest;
  g_pending = true;
  g_eta_ms = ms;
  g_st.moving = true;
  g_st.has_target = true;
  g_st.target_mm = dest;
  g_st.vel_mm_s = (dest >= g_st.pos_mm) ? v : -v;
  g_st.acc_mm_s2 = s->accel_mm_s2;
  refresh_state();
}

bool motion_enable(bool on) {
  g_st.enabled = on;
  if (!on) {
    g_st.moving = false;
    g_st.homing = false;
    g_pending = false;
    g_st.vel_mm_s = 0.0f;
    g_st.acc_mm_s2 = 0.0f;
    g_st.has_target = false;
  }
  refresh_state();
  return true;
}

bool motion_move_to(float mm) {
  if (!g_st.enabled) {
    return false;
  }
  if (g_st.drv_error) {
    return false;
  }
  if (!soft_ok(mm)) {
    return false;
  }
  start_move(mm);
  return true;
}

bool motion_move_by(float mm) { return motion_move_to(g_st.pos_mm + mm); }

bool motion_jog(int dir) {
  if (!g_st.enabled || g_st.drv_error) {
    return false;
  }
  const McConfig *c = config_get();
  float dest = g_st.pos_mm + (dir >= 0 ? 1.0f : -1.0f) * c->max_speed_mm_s * 10.0f;
  if (!isnan(c->slider_max_mm) && dest > c->slider_max_mm) {
    dest = c->slider_max_mm;
  }
  if (!isnan(c->slider_min_mm) && dest < c->slider_min_mm) {
    dest = c->slider_min_mm;
  }
  return motion_move_to(dest);
}

bool motion_stop(void) {
  if (g_pending) {
    g_st.pos_mm = g_pending_target; /* stub: arrive immediately on stop request end */
  }
  g_pending = false;
  g_st.moving = false;
  g_st.homing = false;
  g_st.has_target = false;
  g_st.vel_mm_s = 0.0f;
  g_st.acc_mm_s2 = 0.0f;
  refresh_state();
  return true;
}

bool motion_halt(void) {
  g_pending = false;
  g_st.moving = false;
  g_st.homing = false;
  g_st.has_target = false;
  g_st.vel_mm_s = 0.0f;
  g_st.acc_mm_s2 = 0.0f;
  g_st.enabled = false;
  refresh_state();
  return true;
}

bool motion_home(void) {
  if (!g_st.enabled) {
    return false;
  }
  const McConfig *c = config_get();
  if (c->home_mode == 0) {
    return true;
  }
  bool cfg_ok = false;
  if (c->home_mode == 1 || c->home_mode == 2) {
    cfg_ok = c->sw_home_use != 0;
  } else if (c->home_mode == 3) {
    cfg_ok = c->sw_limit_l_use != 0;
  } else if (c->home_mode == 4) {
    cfg_ok = c->sw_limit_r_use != 0;
  }
  if (!cfg_ok) {
    return false;
  }
  /* Host stub: complete at slider_min/slider_max after a short virtual home. */
  float target;
  if (c->home_mode == 1 || c->home_mode == 3) {
    target = isnan(c->slider_min_mm) ? 0.0f : c->slider_min_mm;
  } else {
    target = isnan(c->slider_max_mm) ? 0.0f : c->slider_max_mm;
  }
  float v = c->home_speed_mm_s;
  if (v < 0.001f) {
    v = 0.001f;
  }
  g_st.homing = true;
  g_st.moving = true;
  g_st.has_target = true;
  g_st.target_mm = target;
  g_pending = true;
  g_pending_target = target;
  float dist = fabsf(target - g_st.pos_mm) + c->home_move_out_mm;
  unsigned ms = (unsigned)(dist / v * 1000.0f);
  if (ms < 50) {
    ms = 50;
  }
  g_eta_ms = ms;
  g_st.vel_mm_s = (target >= g_st.pos_mm) ? v : -v;
  g_st.acc_mm_s2 = c->home_accel_mm_s2;
  refresh_state();
  return true;
}

bool motion_soft_reset(void) {
  g_st.drv_error = false;
  g_pending = false;
  g_st.moving = false;
  g_st.homing = false;
  g_st.has_target = false;
  g_st.vel_mm_s = 0.0f;
  g_st.acc_mm_s2 = 0.0f;
  refresh_state();
  return true;
}

bool motion_set_speed(float mm_s) {
  session_get()->speed_mm_s = mm_s;
  return true;
}

bool motion_set_accel(float mm_s2) {
  session_get()->accel_mm_s2 = mm_s2;
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

void motion_stub_tick_ms(unsigned ms) {
  if (!g_pending) {
    return;
  }
  if (ms >= g_eta_ms) {
    g_eta_ms = 0;
    g_st.pos_mm = g_pending_target;
    g_pending = false;
    g_st.moving = false;
    g_st.homing = false;
    g_st.has_target = false;
    g_st.vel_mm_s = 0.0f;
    g_st.acc_mm_s2 = 0.0f;
    refresh_state();
  } else {
    g_eta_ms -= ms;
  }
}
