#include "config_store.h"
#include "config_defaults.h"

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
}

void session_reset_speed(void) { g_session.speed_mm_s = g_cfg.init_speed_mm_s; }
void session_reset_accel(void) { g_session.accel_mm_s2 = g_cfg.init_accel_mm_s2; }
void session_reset_terminal(void) { g_session.terminal = g_cfg.init_terminal; }
void session_reset_verbose(void) { g_session.verbose = g_cfg.init_verbose; }

void config_init_defaults(void) {
  if (g_cfg_ready) {
    return;
  }
  g_cfg_ready = true;
  g_cfg.init_speed_mm_s = CFG_DEFAULT_INIT_SPEED_MM_S;
  g_cfg.init_accel_mm_s2 = CFG_DEFAULT_INIT_ACCEL_MM_S2;
  g_cfg.max_speed_mm_s = CFG_DEFAULT_MAX_SPEED_MM_S;
  g_cfg.max_accel_mm_s2 = CFG_DEFAULT_MAX_ACCEL_MM_S2;
  g_cfg.steps_per_mm = CFG_DEFAULT_STEPS_PER_MM;
  g_cfg.slider_min_mm = CFG_DEFAULT_SLIDER_MIN_MM;
  g_cfg.slider_max_mm = CFG_DEFAULT_SLIDER_MAX_MM;
  g_cfg.init_verbose = CFG_DEFAULT_INIT_VERBOSE;
  g_cfg.verbose_rate_hz = CFG_DEFAULT_VERBOSE_RATE_HZ;
  g_cfg.init_terminal = CFG_DEFAULT_INIT_TERMINAL;
  g_cfg.init_debug_level = CFG_DEFAULT_INIT_DEBUG_LEVEL;
  g_cfg.wdt_use = CFG_DEFAULT_WDT_USE;
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
  for (int e = 0; e < 10; ++e) {
    g_cfg.ext_active[e] = CFG_DEFAULT_EXT_ACTIVE;
  }
  g_cfg.home_mode = CFG_DEFAULT_HOME_MODE;
  g_cfg.home_move_out_mm = CFG_DEFAULT_HOME_MOVE_OUT_MM;
  g_cfg.home_speed_mm_s = CFG_DEFAULT_HOME_SPEED_MM_S;
  g_cfg.home_accel_mm_s2 = CFG_DEFAULT_HOME_ACCEL_MM_S2;
  g_cfg.ramp_start_hz = CFG_DEFAULT_RAMP_START_HZ;
  g_cfg.stop_approach_hz = CFG_DEFAULT_STOP_APPROACH_HZ;
  g_cfg.dir_change_pause_s = CFG_DEFAULT_DIR_CHANGE_PAUSE_S;
  session_sync_from_config();
}

McConfig *config_get(void) { return &g_cfg; }
McSession *session_get(void) { return &g_session; }

bool config_slider_min_enabled(void) { return !isnan(g_cfg.slider_min_mm); }
bool config_slider_max_enabled(void) { return !isnan(g_cfg.slider_max_mm); }

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
  if (icmp(key, "steps_per_mm") == 0) {
    if (!parse_float(value, &f) || f <= 0.0f) {
      return false;
    }
    g_cfg.steps_per_mm = f;
    return true;
  }
  if (key_is(key, "slider_min", "soft_min")) {
    if (is_none_token(value)) {
      g_cfg.slider_min_mm = NAN;
      return true;
    }
    if (!parse_float(value, &f)) {
      return false;
    }
    g_cfg.slider_min_mm = f;
    return true;
  }
  if (key_is(key, "slider_max", "soft_max")) {
    if (is_none_token(value)) {
      g_cfg.slider_max_mm = NAN;
      return true;
    }
    if (!parse_float(value, &f)) {
      return false;
    }
    g_cfg.slider_max_mm = f;
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
  /* EXT_0_active … EXT_9_active */
  if ((key[0] == 'E' || key[0] == 'e') && (key[1] == 'X' || key[1] == 'x') &&
      (key[2] == 'T' || key[2] == 't') && key[3] == '_' && key[4] >= '0' && key[4] <= '9' &&
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
  if (icmp(key, "steps_per_mm") == 0) {
    snprintf(out, out_len, "%.6g", (double)c->steps_per_mm);
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
  if ((key[0] == 'E' || key[0] == 'e') && (key[1] == 'X' || key[1] == 'x') &&
      (key[2] == 'T' || key[2] == 't') && key[3] == '_' && key[4] >= '0' && key[4] <= '9' &&
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
  return false;
}

void config_foreach(config_foreach_fn fn, void *ctx) {
  static const char *keys[] = {
      "max_speed",
      "max_accel",
      "init_speed",
      "init_accel",
      "steps_per_mm",
      "slider_min",
      "slider_max",
      "init_verbose",
      "verbose_rate_hz",
      "init_terminal",
      "init_debug_level",
      "WDT_use",
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
      "EXT_4_active",
      "EXT_5_active",
      "EXT_6_active",
      "EXT_7_active",
      "EXT_8_active",
      "EXT_9_active",
      "home_mode",
      "home_move_out",
      "home_speed",
      "home_accel",
      "ramp_start_hz",
      "stop_approach_hz",
      "dir_change_pause_s",
  };
  char val[CFG_VAL_MAX];
  for (size_t i = 0; i < sizeof(keys) / sizeof(keys[0]); ++i) {
    if (config_get_key(keys[i], val, sizeof(val))) {
      fn(keys[i], val, ctx);
    }
  }
}
