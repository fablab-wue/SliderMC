#include "config_store.h"
#include "config_defaults.h"
#include "pins.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static McConfig g_cfg;
static McSession g_session;
static bool g_cfg_ready;

static int icmp(const char *a, const char *b) {
  while (*a && *b) {
    char ca = *a;
    char cb = *b;
    if (ca >= 'A' && ca <= 'Z') {
      ca = (char)(ca - 'A' + 'a');
    }
    if (cb >= 'A' && cb <= 'Z') {
      cb = (char)(cb - 'A' + 'a');
    }
    if (ca != cb) {
      return (int)(unsigned char)ca - (int)(unsigned char)cb;
    }
    ++a;
    ++b;
  }
  return (int)(unsigned char)*a - (int)(unsigned char)*b;
}

static bool key_is(const char *key, const char *primary, const char *alias) {
  if (icmp(key, primary) == 0) {
    return true;
  }
  return alias && icmp(key, alias) == 0;
}

void session_sync_from_config(void) {
  g_session.speed_mm_s = g_cfg.init_speed_mm_s;
  g_session.accel_mm_s2 = g_cfg.init_accel_mm_s2;
  g_session.terminal = g_cfg.init_terminal;
  g_session.verbose = g_cfg.init_verbose;
  g_session.path_slice_us = g_cfg.init_path_slice_us;
  g_session.soft_left_mm = g_cfg.slider_min_mm;
  g_session.soft_right_mm = g_cfg.slider_max_mm;
  g_session.soft_left_mm_2 = g_cfg.slider_min_mm_2;
  g_session.soft_right_mm_2 = g_cfg.slider_max_mm_2;
}

void session_reset_speed(void) { g_session.speed_mm_s = g_cfg.init_speed_mm_s; }
void session_reset_accel(void) { g_session.accel_mm_s2 = g_cfg.init_accel_mm_s2; }
void session_reset_terminal(void) { g_session.terminal = g_cfg.init_terminal; }
void session_reset_verbose(void) { g_session.verbose = g_cfg.init_verbose; }
void session_reset_path_slice(void) { g_session.path_slice_us = g_cfg.init_path_slice_us; }
void session_reset_left(void) {
  g_session.soft_left_mm = g_cfg.slider_min_mm;
  g_session.soft_left_mm_2 = g_cfg.slider_min_mm_2;
}
void session_reset_right(void) {
  g_session.soft_right_mm = g_cfg.slider_max_mm;
  g_session.soft_right_mm_2 = g_cfg.slider_max_mm_2;
}

static void clamp_axis_window(float env_min, float env_max, float *left, float *right) {
  if (!isnan(env_min) && !isnan(*left) && *left < env_min) {
    *left = env_min;
  }
  if (!isnan(env_max) && !isnan(*right) && *right > env_max) {
    *right = env_max;
  }
  if (!isnan(*left) && !isnan(*right) && *left > *right) {
    *right = *left;
  }
}

void session_clamp_window_to_envelope(void) {
  clamp_axis_window(g_cfg.slider_min_mm, g_cfg.slider_max_mm, &g_session.soft_left_mm,
                    &g_session.soft_right_mm);
  clamp_axis_window(g_cfg.slider_min_mm_2, g_cfg.slider_max_mm_2, &g_session.soft_left_mm_2,
                    &g_session.soft_right_mm_2);
}

static void init_axis2_defaults(void) {
  g_cfg.steps_per_unit_2 = CFG_DEFAULT_STEPS_PER_UNIT;
  g_cfg.slider_min_mm_2 = CFG_DEFAULT_SLIDER_MIN_MM;
  g_cfg.slider_max_mm_2 = CFG_DEFAULT_SLIDER_MAX_MM;
  g_cfg.drv_step_active_2 = CFG_DEFAULT_DRV_STEP_ACTIVE;
  g_cfg.drv_dir_active_2 = CFG_DEFAULT_DRV_DIR_ACTIVE;
  g_cfg.drv_en_active_2 = CFG_DEFAULT_DRV_EN_ACTIVE;
  g_cfg.drv_error_active_2 = CFG_DEFAULT_DRV_ERROR_ACTIVE;
  g_cfg.sw_home_active_2 = CFG_DEFAULT_SW_HOME_ACTIVE;
  g_cfg.sw_home_use_2 = CFG_DEFAULT_SW_HOME_USE;
  g_cfg.sw_limit_l_active_2 = CFG_DEFAULT_SW_LIMIT_L_ACTIVE;
  g_cfg.sw_limit_r_active_2 = CFG_DEFAULT_SW_LIMIT_R_ACTIVE;
  g_cfg.sw_limit_l_use_2 = CFG_DEFAULT_SW_LIMIT_L_USE;
  g_cfg.sw_limit_r_use_2 = CFG_DEFAULT_SW_LIMIT_R_USE;
  g_cfg.home_mode_2 = CFG_DEFAULT_HOME_MODE;
  g_cfg.home_move_out_mm_2 = CFG_DEFAULT_HOME_MOVE_OUT_MM;
  g_cfg.home_speed_mm_s_2 = CFG_DEFAULT_HOME_SPEED_MM_S;
  g_cfg.home_accel_mm_s2_2 = CFG_DEFAULT_HOME_ACCEL_MM_S2;
  g_cfg.max_speed_mm_s_2 = CFG_DEFAULT_MAX_SPEED_MM_S;
  g_cfg.max_accel_mm_s2_2 = CFG_DEFAULT_MAX_ACCEL_MM_S2;
}

static void config_apply_defaults(void) {
  g_cfg.init_speed_mm_s = CFG_DEFAULT_INIT_SPEED_MM_S;
  g_cfg.init_accel_mm_s2 = CFG_DEFAULT_INIT_ACCEL_MM_S2;
  g_cfg.max_speed_mm_s = CFG_DEFAULT_MAX_SPEED_MM_S;
  g_cfg.max_accel_mm_s2 = CFG_DEFAULT_MAX_ACCEL_MM_S2;
  g_cfg.steps_per_unit = CFG_DEFAULT_STEPS_PER_UNIT;
  g_cfg.slider_min_mm = CFG_DEFAULT_SLIDER_MIN_MM;
  g_cfg.slider_max_mm = CFG_DEFAULT_SLIDER_MAX_MM;
  g_cfg.init_verbose = CFG_DEFAULT_INIT_VERBOSE;
  g_cfg.verbose_rate_hz = CFG_DEFAULT_VERBOSE_RATE_HZ;
  g_cfg.init_terminal = CFG_DEFAULT_INIT_TERMINAL;
  g_cfg.init_debug_level = CFG_DEFAULT_INIT_DEBUG_LEVEL;
  g_cfg.wdt_use = CFG_DEFAULT_WDT_USE;
  g_cfg.axis2_use = CFG_DEFAULT_AXIS2_USE;
  g_cfg.name[0] = 0;
  strncpy(g_cfg.unit_name, CFG_DEFAULT_UNIT_NAME, sizeof(g_cfg.unit_name) - 1);
  g_cfg.unit_name[sizeof(g_cfg.unit_name) - 1] = 0;
  g_cfg.drv_step_active = CFG_DEFAULT_DRV_STEP_ACTIVE;
  g_cfg.drv_dir_active = CFG_DEFAULT_DRV_DIR_ACTIVE;
  g_cfg.drv_en_active = CFG_DEFAULT_DRV_EN_ACTIVE;
  g_cfg.sw_home_active = CFG_DEFAULT_SW_HOME_ACTIVE;
  g_cfg.sw_home_use = CFG_DEFAULT_SW_HOME_USE;
  g_cfg.drv_error_active = CFG_DEFAULT_DRV_ERROR_ACTIVE;
  g_cfg.sw_limit_l_active = CFG_DEFAULT_SW_LIMIT_L_ACTIVE;
  g_cfg.sw_limit_r_active = CFG_DEFAULT_SW_LIMIT_R_ACTIVE;
  g_cfg.sw_limit_l_use = CFG_DEFAULT_SW_LIMIT_L_USE;
  g_cfg.sw_limit_r_use = CFG_DEFAULT_SW_LIMIT_R_USE;
  for (int e = 0; e < 4; ++e) {
    g_cfg.ext_active[e] = CFG_DEFAULT_EXT_ACTIVE;
  }
  g_cfg.home_mode = CFG_DEFAULT_HOME_MODE;
  g_cfg.home_move_out_mm = CFG_DEFAULT_HOME_MOVE_OUT_MM;
  g_cfg.home_speed_mm_s = CFG_DEFAULT_HOME_SPEED_MM_S;
  g_cfg.home_accel_mm_s2 = CFG_DEFAULT_HOME_ACCEL_MM_S2;
  init_axis2_defaults();
  g_cfg.ramp_start_hz = CFG_DEFAULT_RAMP_START_HZ;
  g_cfg.stop_approach_hz = CFG_DEFAULT_STOP_APPROACH_HZ;
  g_cfg.dir_change_pause_s = CFG_DEFAULT_DIR_CHANGE_PAUSE_S;
  g_cfg.path_buffer_size = CFG_DEFAULT_PATH_BUFFER_SIZE;
  if (g_cfg.path_buffer_size > PATH_BUFFER_MAX) {
    g_cfg.path_buffer_size = PATH_BUFFER_MAX;
  }
  g_cfg.init_path_slice_us = CFG_DEFAULT_INIT_PATH_SLICE_US;
  session_sync_from_config();
}

void config_init_defaults(void) {
  if (g_cfg_ready) {
    return;
  }
  g_cfg_ready = true;
  config_apply_defaults();
}

void config_reset_to_defaults(void) {
  g_cfg_ready = true;
  config_apply_defaults();
}

McConfig *config_get(void) { return &g_cfg; }
McSession *session_get(void) { return &g_session; }

bool config_slider_min_enabled(void) { return !isnan(g_cfg.slider_min_mm); }
bool config_slider_max_enabled(void) { return !isnan(g_cfg.slider_max_mm); }

bool config_axis2_enabled(void) {
#if PIN_AXIS2_SUPPORTED
  return g_cfg.axis2_use != 0;
#else
  if (g_cfg.axis2_use != 0) {
    g_cfg.axis2_use = 0;
  }
  return false;
#endif
}

static bool pos_in_envelope(int axis, float pos) {
  float mn = (axis == 1) ? g_cfg.slider_min_mm_2 : g_cfg.slider_min_mm;
  float mx = (axis == 1) ? g_cfg.slider_max_mm_2 : g_cfg.slider_max_mm;
  if (!isnan(mn) && pos < mn - 1e-4f) {
    return false;
  }
  if (!isnan(mx) && pos > mx + 1e-4f) {
    return false;
  }
  return true;
}

bool session_set_window_left(float mm0_or_nan, float mm1_or_nan) {
  if (!isnan(mm0_or_nan)) {
    if (!pos_in_envelope(0, mm0_or_nan)) {
      return false;
    }
    if (!isnan(g_session.soft_right_mm) && mm0_or_nan > g_session.soft_right_mm + 1e-4f) {
      return false;
    }
  }
  if (!isnan(mm1_or_nan) && config_axis2_enabled()) {
    if (!pos_in_envelope(1, mm1_or_nan)) {
      return false;
    }
    if (!isnan(g_session.soft_right_mm_2) &&
        mm1_or_nan > g_session.soft_right_mm_2 + 1e-4f) {
      return false;
    }
  }
  if (!isnan(mm0_or_nan)) {
    g_session.soft_left_mm = mm0_or_nan;
  }
  if (!isnan(mm1_or_nan) && config_axis2_enabled()) {
    g_session.soft_left_mm_2 = mm1_or_nan;
  }
  return true;
}

bool session_set_window_right(float mm0_or_nan, float mm1_or_nan) {
  if (!isnan(mm0_or_nan)) {
    if (!pos_in_envelope(0, mm0_or_nan)) {
      return false;
    }
    if (!isnan(g_session.soft_left_mm) && mm0_or_nan < g_session.soft_left_mm - 1e-4f) {
      return false;
    }
  }
  if (!isnan(mm1_or_nan) && config_axis2_enabled()) {
    if (!pos_in_envelope(1, mm1_or_nan)) {
      return false;
    }
    if (!isnan(g_session.soft_left_mm_2) &&
        mm1_or_nan < g_session.soft_left_mm_2 - 1e-4f) {
      return false;
    }
  }
  if (!isnan(mm0_or_nan)) {
    g_session.soft_right_mm = mm0_or_nan;
  }
  if (!isnan(mm1_or_nan) && config_axis2_enabled()) {
    g_session.soft_right_mm_2 = mm1_or_nan;
  }
  return true;
}

bool config_pin_asserted(int gpio_level, int active) {
  int level = gpio_level ? 1 : 0;
  int act = active ? 1 : 0;
  return level == act;
}

static bool parse_float(const char *s, float *out) {
  if (!s || !*s) {
    return false;
  }
  char *end = nullptr;
  float v = strtof(s, &end);
  if (end == s) {
    return false;
  }
  *out = v;
  return true;
}

static bool parse_int(const char *s, int *out) {
  if (!s || !*s) {
    return false;
  }
  char *end = nullptr;
  long v = strtol(s, &end, 10);
  if (end == s) {
    return false;
  }
  *out = (int)v;
  return true;
}

static bool is_none_token(const char *s) {
  return icmp(s, "-") == 0 || icmp(s, "none") == 0;
}

static bool set_01(const char *value, int *field) {
  int i;
  if (!parse_int(value, &i) || (i != 0 && i != 1)) {
    return false;
  }
  *field = i;
  return true;
}

bool config_set_key(const char *key, const char *value) {
  if (!key || !value) {
    return false;
  }
  float f;
  int i;

  if (key_is(key, "init_speed", "speed")) {
    if (!parse_float(value, &f) || f < 0.0f || f > g_cfg.max_speed_mm_s) {
      return false;
    }
    g_cfg.init_speed_mm_s = f;
    g_session.speed_mm_s = f;
    return true;
  }
  if (key_is(key, "init_accel", "accel")) {
    if (!parse_float(value, &f) || f <= 0.0f || f > g_cfg.max_accel_mm_s2) {
      return false;
    }
    g_cfg.init_accel_mm_s2 = f;
    g_session.accel_mm_s2 = f;
    return true;
  }
  if (icmp(key, "max_speed") == 0) {
    if (!parse_float(value, &f) || f <= 0.0f) {
      return false;
    }
    g_cfg.max_speed_mm_s = f;
    return true;
  }
  if (icmp(key, "max_accel") == 0) {
    if (!parse_float(value, &f) || f <= 0.0f) {
      return false;
    }
    g_cfg.max_accel_mm_s2 = f;
    return true;
  }
  if (icmp(key, "max_speed_2") == 0) {
    if (!parse_float(value, &f) || f <= 0.0f) {
      return false;
    }
    g_cfg.max_speed_mm_s_2 = f;
    return true;
  }
  if (icmp(key, "max_accel_2") == 0) {
    if (!parse_float(value, &f) || f <= 0.0f) {
      return false;
    }
    g_cfg.max_accel_mm_s2_2 = f;
    return true;
  }
  if (key_is(key, "steps_per_unit", "steps_per_mm")) {
    if (!parse_float(value, &f) || f <= 0.0f) {
      return false;
    }
    g_cfg.steps_per_unit = f;
    return true;
  }
  if (key_is(key, "steps_per_unit_2", "steps_per_mm_2")) {
    if (!parse_float(value, &f) || f <= 0.0f) {
      return false;
    }
    g_cfg.steps_per_unit_2 = f;
    return true;
  }
  if (key_is(key, "slider_min", "soft_min")) {
    if (is_none_token(value)) {
      g_cfg.slider_min_mm = NAN;
      session_clamp_window_to_envelope();
      return true;
    }
    if (!parse_float(value, &f)) {
      return false;
    }
    g_cfg.slider_min_mm = f;
    session_clamp_window_to_envelope();
    return true;
  }
  if (key_is(key, "slider_max", "soft_max")) {
    if (is_none_token(value)) {
      g_cfg.slider_max_mm = NAN;
      session_clamp_window_to_envelope();
      return true;
    }
    if (!parse_float(value, &f)) {
      return false;
    }
    g_cfg.slider_max_mm = f;
    session_clamp_window_to_envelope();
    return true;
  }
  if (key_is(key, "slider_min_2", "soft_min_2")) {
    if (is_none_token(value)) {
      g_cfg.slider_min_mm_2 = NAN;
      session_clamp_window_to_envelope();
      return true;
    }
    if (!parse_float(value, &f)) {
      return false;
    }
    g_cfg.slider_min_mm_2 = f;
    session_clamp_window_to_envelope();
    return true;
  }
  if (key_is(key, "slider_max_2", "soft_max_2")) {
    if (is_none_token(value)) {
      g_cfg.slider_max_mm_2 = NAN;
      session_clamp_window_to_envelope();
      return true;
    }
    if (!parse_float(value, &f)) {
      return false;
    }
    g_cfg.slider_max_mm_2 = f;
    session_clamp_window_to_envelope();
    return true;
  }
  if (key_is(key, "init_verbose", "verbose")) {
    if (!set_01(value, &g_cfg.init_verbose)) {
      return false;
    }
    g_session.verbose = g_cfg.init_verbose;
    return true;
  }
  if (icmp(key, "verbose_rate_hz") == 0) {
    if (!parse_int(value, &i) || i < 1 || i > 200) {
      return false;
    }
    g_cfg.verbose_rate_hz = i;
    return true;
  }
  if (key_is(key, "init_terminal", "terminal")) {
    if (!set_01(value, &g_cfg.init_terminal)) {
      return false;
    }
    g_session.terminal = g_cfg.init_terminal;
    return true;
  }
  if (key_is(key, "init_debug_level", "debug_level")) {
    if (!parse_int(value, &i) || i < 0 || i > 5) {
      return false;
    }
    g_cfg.init_debug_level = i;
    return true;
  }
  if (icmp(key, "WDT_use") == 0) {
    return set_01(value, &g_cfg.wdt_use);
  }
  if (icmp(key, "axis2_use") == 0) {
    if (!set_01(value, &i)) {
      return false;
    }
#if !PIN_AXIS2_SUPPORTED
    g_cfg.axis2_use = 0;
    if (i != 0) {
      return false;
    }
    return true;
#else
    g_cfg.axis2_use = i;
    return true;
#endif
  }
  if (icmp(key, "name") == 0) {
    /* Printable ASCII, no '#' or control chars; empty clears. */
    size_t n = 0;
    if (!value) {
      value = "";
    }
    while (value[n] && n + 1 < sizeof(g_cfg.name)) {
      unsigned char ch = (unsigned char)value[n];
      if (ch < 0x20 || ch > 0x7e || ch == '#') {
        return false;
      }
      ++n;
    }
    if (value[n] != 0) {
      return false; /* too long */
    }
    memcpy(g_cfg.name, value, n);
    g_cfg.name[n] = 0;
    return true;
  }
  if (icmp(key, "unit_name") == 0) {
    /* Printable ASCII label for UIC (e.g. mm, deg); non-empty. */
    size_t n = 0;
    if (!value || !value[0]) {
      return false;
    }
    while (value[n] && n + 1 < sizeof(g_cfg.unit_name)) {
      unsigned char ch = (unsigned char)value[n];
      if (ch < 0x20 || ch > 0x7e || ch == '#') {
        return false;
      }
      ++n;
    }
    if (value[n] != 0) {
      return false; /* too long */
    }
    memcpy(g_cfg.unit_name, value, n);
    g_cfg.unit_name[n] = 0;
    return true;
  }
  if (icmp(key, "DRV_STEP_active") == 0) {
    return set_01(value, &g_cfg.drv_step_active);
  }
  if (icmp(key, "DRV_DIR_active") == 0) {
    return set_01(value, &g_cfg.drv_dir_active);
  }
  if (icmp(key, "DRV_EN_active") == 0) {
    return set_01(value, &g_cfg.drv_en_active);
  }
  if (icmp(key, "SW_HOME_active") == 0) {
    return set_01(value, &g_cfg.sw_home_active);
  }
  if (icmp(key, "SW_HOME_use") == 0) {
    return set_01(value, &g_cfg.sw_home_use);
  }
  if (icmp(key, "DRV_ERROR_active") == 0) {
    return set_01(value, &g_cfg.drv_error_active);
  }
  if (icmp(key, "SW_LIMIT_L_active") == 0) {
    return set_01(value, &g_cfg.sw_limit_l_active);
  }
  if (icmp(key, "SW_LIMIT_R_active") == 0) {
    return set_01(value, &g_cfg.sw_limit_r_active);
  }
  if (icmp(key, "SW_LIMIT_L_use") == 0) {
    return set_01(value, &g_cfg.sw_limit_l_use);
  }
  if (icmp(key, "SW_LIMIT_R_use") == 0) {
    return set_01(value, &g_cfg.sw_limit_r_use);
  }
  if (icmp(key, "DRV_STEP_active_2") == 0) {
    return set_01(value, &g_cfg.drv_step_active_2);
  }
  if (icmp(key, "DRV_DIR_active_2") == 0) {
    return set_01(value, &g_cfg.drv_dir_active_2);
  }
  if (icmp(key, "DRV_EN_active_2") == 0) {
    return set_01(value, &g_cfg.drv_en_active_2);
  }
  if (icmp(key, "DRV_ERROR_active_2") == 0) {
    return set_01(value, &g_cfg.drv_error_active_2);
  }
  if (icmp(key, "SW_HOME_active_2") == 0) {
    return set_01(value, &g_cfg.sw_home_active_2);
  }
  if (icmp(key, "SW_HOME_use_2") == 0) {
    return set_01(value, &g_cfg.sw_home_use_2);
  }
  if (icmp(key, "SW_LIMIT_L_active_2") == 0) {
    return set_01(value, &g_cfg.sw_limit_l_active_2);
  }
  if (icmp(key, "SW_LIMIT_R_active_2") == 0) {
    return set_01(value, &g_cfg.sw_limit_r_active_2);
  }
  if (icmp(key, "SW_LIMIT_L_use_2") == 0) {
    return set_01(value, &g_cfg.sw_limit_l_use_2);
  }
  if (icmp(key, "SW_LIMIT_R_use_2") == 0) {
    return set_01(value, &g_cfg.sw_limit_r_use_2);
  }
  /* EXT_0_active … EXT_3_active */
  if ((key[0] == 'E' || key[0] == 'e') && (key[1] == 'X' || key[1] == 'x') &&
      (key[2] == 'T' || key[2] == 't') && key[3] == '_' && key[4] >= '0' && key[4] <= '3' &&
      key[5] == '_' && icmp(key + 6, "active") == 0) {
    return set_01(value, &g_cfg.ext_active[key[4] - '0']);
  }
  if (icmp(key, "home_mode") == 0) {
    if (!parse_int(value, &i) || i < 0 || i > 4) {
      return false;
    }
    g_cfg.home_mode = i;
    return true;
  }
  if (icmp(key, "home_move_out") == 0) {
    if (!parse_float(value, &f) || f < 0.0f) {
      return false;
    }
    g_cfg.home_move_out_mm = f;
    return true;
  }
  if (icmp(key, "home_speed") == 0) {
    if (!parse_float(value, &f) || f <= 0.0f) {
      return false;
    }
    g_cfg.home_speed_mm_s = f;
    return true;
  }
  if (icmp(key, "home_accel") == 0) {
    if (!parse_float(value, &f) || f <= 0.0f) {
      return false;
    }
    g_cfg.home_accel_mm_s2 = f;
    return true;
  }
  if (icmp(key, "home_mode_2") == 0) {
    if (!parse_int(value, &i) || i < 0 || i > 4) {
      return false;
    }
    g_cfg.home_mode_2 = i;
    return true;
  }
  if (icmp(key, "home_move_out_2") == 0 || icmp(key, "home_move_out_mm_2") == 0) {
    if (!parse_float(value, &f) || f < 0.0f) {
      return false;
    }
    g_cfg.home_move_out_mm_2 = f;
    return true;
  }
  if (icmp(key, "home_speed_2") == 0 || icmp(key, "home_speed_mm_s_2") == 0) {
    if (!parse_float(value, &f) || f <= 0.0f) {
      return false;
    }
    g_cfg.home_speed_mm_s_2 = f;
    return true;
  }
  if (icmp(key, "home_accel_2") == 0 || icmp(key, "home_accel_mm_s2_2") == 0) {
    if (!parse_float(value, &f) || f <= 0.0f) {
      return false;
    }
    g_cfg.home_accel_mm_s2_2 = f;
    return true;
  }
  if (icmp(key, "ramp_start_hz") == 0) {
    if (!parse_int(value, &i) || i < 0) {
      return false;
    }
    g_cfg.ramp_start_hz = i;
    return true;
  }
  if (icmp(key, "stop_approach_hz") == 0) {
    if (!parse_int(value, &i) || i < 0) {
      return false;
    }
    g_cfg.stop_approach_hz = i;
    return true;
  }
  if (icmp(key, "dir_change_pause_s") == 0) {
    if (!parse_float(value, &f) || f < 0.0f) {
      return false;
    }
    g_cfg.dir_change_pause_s = f;
    return true;
  }
  if (icmp(key, "path_buffer_size") == 0) {
    if (!parse_int(value, &i) || i < 1 || i > PATH_BUFFER_MAX) {
      return false;
    }
    g_cfg.path_buffer_size = i;
    return true;
  }
  if (icmp(key, "init_path_slice_us") == 0) {
    if (!parse_int(value, &i) || i < PATH_SLICE_US_MIN) {
      return false;
    }
    g_cfg.init_path_slice_us = i;
    g_session.path_slice_us = i;
    return true;
  }
  return false;
}

bool config_get_key(const char *key, char *out, size_t out_len) {
  if (!key || !out || out_len < 2) {
    return false;
  }
  const McConfig *c = &g_cfg;
  if (key_is(key, "init_speed", "speed")) {
    snprintf(out, out_len, "%.3g", (double)c->init_speed_mm_s);
    return true;
  }
  if (key_is(key, "init_accel", "accel")) {
    snprintf(out, out_len, "%.3g", (double)c->init_accel_mm_s2);
    return true;
  }
  if (icmp(key, "max_speed") == 0) {
    snprintf(out, out_len, "%.3g", (double)c->max_speed_mm_s);
    return true;
  }
  if (icmp(key, "max_accel") == 0) {
    snprintf(out, out_len, "%.3g", (double)c->max_accel_mm_s2);
    return true;
  }
  if (icmp(key, "max_speed_2") == 0) {
    snprintf(out, out_len, "%.3g", (double)c->max_speed_mm_s_2);
    return true;
  }
  if (icmp(key, "max_accel_2") == 0) {
    snprintf(out, out_len, "%.3g", (double)c->max_accel_mm_s2_2);
    return true;
  }
  if (key_is(key, "steps_per_unit", "steps_per_mm")) {
    snprintf(out, out_len, "%.6g", (double)c->steps_per_unit);
    return true;
  }
  if (key_is(key, "steps_per_unit_2", "steps_per_mm_2")) {
    snprintf(out, out_len, "%.6g", (double)c->steps_per_unit_2);
    return true;
  }
  if (key_is(key, "slider_min", "soft_min")) {
    if (isnan(c->slider_min_mm)) {
      snprintf(out, out_len, "none");
    } else {
      snprintf(out, out_len, "%.3g", (double)c->slider_min_mm);
    }
    return true;
  }
  if (key_is(key, "slider_max", "soft_max")) {
    if (isnan(c->slider_max_mm)) {
      snprintf(out, out_len, "none");
    } else {
      snprintf(out, out_len, "%.3g", (double)c->slider_max_mm);
    }
    return true;
  }
  if (key_is(key, "slider_min_2", "soft_min_2")) {
    if (isnan(c->slider_min_mm_2)) {
      snprintf(out, out_len, "none");
    } else {
      snprintf(out, out_len, "%.3g", (double)c->slider_min_mm_2);
    }
    return true;
  }
  if (key_is(key, "slider_max_2", "soft_max_2")) {
    if (isnan(c->slider_max_mm_2)) {
      snprintf(out, out_len, "none");
    } else {
      snprintf(out, out_len, "%.3g", (double)c->slider_max_mm_2);
    }
    return true;
  }
  if (key_is(key, "init_verbose", "verbose")) {
    snprintf(out, out_len, "%d", c->init_verbose);
    return true;
  }
  if (icmp(key, "verbose_rate_hz") == 0) {
    snprintf(out, out_len, "%d", c->verbose_rate_hz);
    return true;
  }
  if (key_is(key, "init_terminal", "terminal")) {
    snprintf(out, out_len, "%d", c->init_terminal);
    return true;
  }
  if (key_is(key, "init_debug_level", "debug_level")) {
    snprintf(out, out_len, "%d", c->init_debug_level);
    return true;
  }
  if (icmp(key, "WDT_use") == 0) {
    snprintf(out, out_len, "%d", c->wdt_use);
    return true;
  }
  if (icmp(key, "axis2_use") == 0) {
    snprintf(out, out_len, "%d", c->axis2_use);
    return true;
  }
  if (icmp(key, "name") == 0) {
    snprintf(out, out_len, "%s", c->name);
    return true;
  }
  if (icmp(key, "unit_name") == 0) {
    snprintf(out, out_len, "%s", c->unit_name);
    return true;
  }
  if (icmp(key, "DRV_STEP_active") == 0) {
    snprintf(out, out_len, "%d", c->drv_step_active);
    return true;
  }
  if (icmp(key, "DRV_DIR_active") == 0) {
    snprintf(out, out_len, "%d", c->drv_dir_active);
    return true;
  }
  if (icmp(key, "DRV_EN_active") == 0) {
    snprintf(out, out_len, "%d", c->drv_en_active);
    return true;
  }
  if (icmp(key, "SW_HOME_active") == 0) {
    snprintf(out, out_len, "%d", c->sw_home_active);
    return true;
  }
  if (icmp(key, "SW_HOME_use") == 0) {
    snprintf(out, out_len, "%d", c->sw_home_use);
    return true;
  }
  if (icmp(key, "DRV_ERROR_active") == 0) {
    snprintf(out, out_len, "%d", c->drv_error_active);
    return true;
  }
  if (icmp(key, "SW_LIMIT_L_active") == 0) {
    snprintf(out, out_len, "%d", c->sw_limit_l_active);
    return true;
  }
  if (icmp(key, "SW_LIMIT_R_active") == 0) {
    snprintf(out, out_len, "%d", c->sw_limit_r_active);
    return true;
  }
  if (icmp(key, "SW_LIMIT_L_use") == 0) {
    snprintf(out, out_len, "%d", c->sw_limit_l_use);
    return true;
  }
  if (icmp(key, "SW_LIMIT_R_use") == 0) {
    snprintf(out, out_len, "%d", c->sw_limit_r_use);
    return true;
  }
  if (icmp(key, "DRV_STEP_active_2") == 0) {
    snprintf(out, out_len, "%d", c->drv_step_active_2);
    return true;
  }
  if (icmp(key, "DRV_DIR_active_2") == 0) {
    snprintf(out, out_len, "%d", c->drv_dir_active_2);
    return true;
  }
  if (icmp(key, "DRV_EN_active_2") == 0) {
    snprintf(out, out_len, "%d", c->drv_en_active_2);
    return true;
  }
  if (icmp(key, "DRV_ERROR_active_2") == 0) {
    snprintf(out, out_len, "%d", c->drv_error_active_2);
    return true;
  }
  if (icmp(key, "SW_HOME_active_2") == 0) {
    snprintf(out, out_len, "%d", c->sw_home_active_2);
    return true;
  }
  if (icmp(key, "SW_HOME_use_2") == 0) {
    snprintf(out, out_len, "%d", c->sw_home_use_2);
    return true;
  }
  if (icmp(key, "SW_LIMIT_L_active_2") == 0) {
    snprintf(out, out_len, "%d", c->sw_limit_l_active_2);
    return true;
  }
  if (icmp(key, "SW_LIMIT_R_active_2") == 0) {
    snprintf(out, out_len, "%d", c->sw_limit_r_active_2);
    return true;
  }
  if (icmp(key, "SW_LIMIT_L_use_2") == 0) {
    snprintf(out, out_len, "%d", c->sw_limit_l_use_2);
    return true;
  }
  if (icmp(key, "SW_LIMIT_R_use_2") == 0) {
    snprintf(out, out_len, "%d", c->sw_limit_r_use_2);
    return true;
  }
  if ((key[0] == 'E' || key[0] == 'e') && (key[1] == 'X' || key[1] == 'x') &&
      (key[2] == 'T' || key[2] == 't') && key[3] == '_' && key[4] >= '0' && key[4] <= '3' &&
      key[5] == '_' && icmp(key + 6, "active") == 0) {
    snprintf(out, out_len, "%d", c->ext_active[key[4] - '0']);
    return true;
  }
  if (icmp(key, "home_mode") == 0) {
    snprintf(out, out_len, "%d", c->home_mode);
    return true;
  }
  if (icmp(key, "home_move_out") == 0) {
    snprintf(out, out_len, "%.3g", (double)c->home_move_out_mm);
    return true;
  }
  if (icmp(key, "home_speed") == 0) {
    snprintf(out, out_len, "%.3g", (double)c->home_speed_mm_s);
    return true;
  }
  if (icmp(key, "home_accel") == 0) {
    snprintf(out, out_len, "%.3g", (double)c->home_accel_mm_s2);
    return true;
  }
  if (icmp(key, "home_mode_2") == 0) {
    snprintf(out, out_len, "%d", c->home_mode_2);
    return true;
  }
  if (icmp(key, "home_move_out_2") == 0 || icmp(key, "home_move_out_mm_2") == 0) {
    snprintf(out, out_len, "%.3g", (double)c->home_move_out_mm_2);
    return true;
  }
  if (icmp(key, "home_speed_2") == 0 || icmp(key, "home_speed_mm_s_2") == 0) {
    snprintf(out, out_len, "%.3g", (double)c->home_speed_mm_s_2);
    return true;
  }
  if (icmp(key, "home_accel_2") == 0 || icmp(key, "home_accel_mm_s2_2") == 0) {
    snprintf(out, out_len, "%.3g", (double)c->home_accel_mm_s2_2);
    return true;
  }
  if (icmp(key, "ramp_start_hz") == 0) {
    snprintf(out, out_len, "%d", c->ramp_start_hz);
    return true;
  }
  if (icmp(key, "stop_approach_hz") == 0) {
    snprintf(out, out_len, "%d", c->stop_approach_hz);
    return true;
  }
  if (icmp(key, "dir_change_pause_s") == 0) {
    snprintf(out, out_len, "%.3g", (double)c->dir_change_pause_s);
    return true;
  }
  if (icmp(key, "path_buffer_size") == 0) {
    snprintf(out, out_len, "%d", c->path_buffer_size);
    return true;
  }
  if (icmp(key, "init_path_slice_us") == 0) {
    snprintf(out, out_len, "%d", c->init_path_slice_us);
    return true;
  }
  return false;
}

void config_foreach(config_foreach_fn fn, void *ctx) {
  static const char *keys[] = {
      "max_speed",
      "max_accel",
      "max_speed_2",
      "max_accel_2",
      "init_speed",
      "init_accel",
      "steps_per_unit",
      "unit_name",
      "slider_min",
      "slider_max",
      "init_verbose",
      "verbose_rate_hz",
      "init_terminal",
      "init_debug_level",
      "WDT_use",
      "axis2_use",
      "name",
      "DRV_STEP_active",
      "DRV_DIR_active",
      "DRV_EN_active",
      "SW_HOME_active",
      "SW_HOME_use",
      "DRV_ERROR_active",
      "SW_LIMIT_L_active",
      "SW_LIMIT_R_active",
      "SW_LIMIT_L_use",
      "SW_LIMIT_R_use",
      "EXT_0_active",
      "EXT_1_active",
      "EXT_2_active",
      "EXT_3_active",
      "home_mode",
      "home_move_out",
      "home_speed",
      "home_accel",
      "steps_per_unit_2",
      "slider_min_2",
      "slider_max_2",
      "DRV_STEP_active_2",
      "DRV_DIR_active_2",
      "DRV_EN_active_2",
      "DRV_ERROR_active_2",
      "SW_HOME_active_2",
      "SW_HOME_use_2",
      "SW_LIMIT_L_active_2",
      "SW_LIMIT_R_active_2",
      "SW_LIMIT_L_use_2",
      "SW_LIMIT_R_use_2",
      "home_mode_2",
      "home_move_out_2",
      "home_speed_2",
      "home_accel_2",
      "ramp_start_hz",
      "stop_approach_hz",
      "dir_change_pause_s",
      "path_buffer_size",
      "init_path_slice_us",
  };
  char val[CFG_VAL_MAX];
  for (size_t i = 0; i < sizeof(keys) / sizeof(keys[0]); ++i) {
    if (config_get_key(keys[i], val, sizeof(val))) {
      fn(keys[i], val, ctx);
    }
  }
}
