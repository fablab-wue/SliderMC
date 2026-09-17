#include "config_store.h"
#include "config_defaults.h"
#include "pins.h"
#include "motion_path.h"

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

static void clamp_cfg_counts(void);

float config_channel_min(int ch);
float config_channel_max(int ch);

void session_sync_from_config(void) {
  g_session.speed_mm_s = g_cfg.init_speed_mm_s;
  g_session.accel_mm_s2 = g_cfg.init_accel_mm_s2;
  g_session.decel_mm_s2 = g_cfg.init_accel_mm_s2;
  g_session.terminal = g_cfg.init_terminal;
  g_session.verbose = g_cfg.init_verbose;
  g_session.path_slice_us = g_cfg.init_path_slice_us;
  for (int i = 0; i < MC_CH_MAX; ++i) {
    g_session.soft_left[i] = config_channel_min(i);
    g_session.soft_right[i] = config_channel_max(i);
  }
}

void session_reset_speed(void) { g_session.speed_mm_s = g_cfg.init_speed_mm_s; }
void session_reset_accel(void) {
  g_session.accel_mm_s2 = g_cfg.init_accel_mm_s2;
  g_session.decel_mm_s2 = g_cfg.init_accel_mm_s2;
}
void session_reset_terminal(void) { g_session.terminal = g_cfg.init_terminal; }
void session_reset_verbose(void) { g_session.verbose = g_cfg.init_verbose; }
void session_reset_path_slice(void) { g_session.path_slice_us = g_cfg.init_path_slice_us; }
void session_reset_left(void) {
  for (int i = 0; i < MC_CH_MAX; ++i) {
    g_session.soft_left[i] = config_channel_min(i);
  }
}
void session_reset_right(void) {
  for (int i = 0; i < MC_CH_MAX; ++i) {
    g_session.soft_right[i] = config_channel_max(i);
  }
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
  for (int i = 0; i < MC_CH_MAX; ++i) {
    clamp_axis_window(config_channel_min(i), config_channel_max(i), &g_session.soft_left[i],
                      &g_session.soft_right[i]);
  }
}

static void init_axis2_defaults(void) {
  g_cfg.steps_per_unit_2 = CFG_DEFAULT_STEPS_PER_UNIT;
  g_cfg.drv_step_active_2 = CFG_DEFAULT_DRV_STEP_ACTIVE;
  g_cfg.drv_dir_active_2 = CFG_DEFAULT_DRV_DIR_ACTIVE;
  g_cfg.drv_error_active_2 = CFG_DEFAULT_DRV_ERROR_ACTIVE;
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

static void init_axis3_defaults(void) {
  g_cfg.steps_per_unit_3 = CFG_DEFAULT_STEPS_PER_UNIT;
  g_cfg.drv_dir_active_3 = CFG_DEFAULT_DRV_DIR_ACTIVE;
  g_cfg.drv_error_active_3 = CFG_DEFAULT_DRV_ERROR_ACTIVE;
  g_cfg.sw_limit_l_active_3 = CFG_DEFAULT_SW_LIMIT_L_ACTIVE;
  g_cfg.sw_limit_r_active_3 = CFG_DEFAULT_SW_LIMIT_R_ACTIVE;
  g_cfg.sw_limit_l_use_3 = CFG_DEFAULT_SW_LIMIT_L_USE;
  g_cfg.sw_limit_r_use_3 = CFG_DEFAULT_SW_LIMIT_R_USE;
  g_cfg.home_mode_3 = CFG_DEFAULT_HOME_MODE;
  g_cfg.home_move_out_mm_3 = CFG_DEFAULT_HOME_MOVE_OUT_MM;
  g_cfg.home_speed_mm_s_3 = CFG_DEFAULT_HOME_SPEED_MM_S;
  g_cfg.home_accel_mm_s2_3 = CFG_DEFAULT_HOME_ACCEL_MM_S2;
  g_cfg.max_speed_mm_s_3 = CFG_DEFAULT_MAX_SPEED_MM_S;
  g_cfg.max_accel_mm_s2_3 = CFG_DEFAULT_MAX_ACCEL_MM_S2;
}

static int axis_board_max(void) {
#if PIN_AXIS3_SUPPORTED
  return 3;
#elif PIN_AXIS2_SUPPORTED
  return 2;
#else
  return 1;
#endif
}

static void clamp_cfg_counts(void) {
  int mx = axis_board_max();
  if (g_cfg.motors < 1) {
    g_cfg.motors = 1;
  }
  if (g_cfg.motors > mx) {
    g_cfg.motors = mx;
  }
  if (g_cfg.servos < 0) {
    g_cfg.servos = 0;
  }
  if (g_cfg.servos > SERVO_MAX) {
    g_cfg.servos = SERVO_MAX;
  }
  int n = g_cfg.motors + g_cfg.servos;
  if (n < 1) {
    n = 1;
  }
  int cap = PATH_POOL_SAMPLES / n;
  if (g_cfg.path_buffer_size > cap) {
    g_cfg.path_buffer_size = cap;
  }
}

static void config_apply_defaults(void) {
  g_cfg.init_speed_mm_s = CFG_DEFAULT_INIT_SPEED_MM_S;
  g_cfg.init_accel_mm_s2 = CFG_DEFAULT_INIT_ACCEL_MM_S2;
  g_cfg.max_speed_mm_s = CFG_DEFAULT_MAX_SPEED_MM_S;
  g_cfg.max_accel_mm_s2 = CFG_DEFAULT_MAX_ACCEL_MM_S2;
  g_cfg.steps_per_unit = CFG_DEFAULT_STEPS_PER_UNIT;
  for (int i = 0; i < MOTOR_MAX; ++i) {
    g_cfg.motor_min[i] = CFG_DEFAULT_MOTOR_MIN;
    g_cfg.motor_max[i] = CFG_DEFAULT_MOTOR_MAX;
  }
  for (int i = 0; i < SERVO_MAX; ++i) {
    g_cfg.servo_min[i] = CFG_DEFAULT_SERVO_MIN;
    g_cfg.servo_max[i] = CFG_DEFAULT_SERVO_MAX;
    g_cfg.servo_max_speed[i] = CFG_DEFAULT_SERVO_MAX_SPEED;
    g_cfg.servo_max_accel[i] = CFG_DEFAULT_SERVO_MAX_ACCEL;
    g_cfg.servo_active[i] = CFG_DEFAULT_SERVO_ACTIVE;
    g_cfg.servo_min_pulse[i] = CFG_DEFAULT_SERVO_MIN_PULSE;
    g_cfg.servo_max_pulse[i] = CFG_DEFAULT_SERVO_MAX_PULSE;
    g_cfg.servo_swap[i] = CFG_DEFAULT_SERVO_SWAP;
  }
  g_cfg.init_verbose = CFG_DEFAULT_INIT_VERBOSE;
  g_cfg.verbose_rate_hz = CFG_DEFAULT_VERBOSE_RATE_HZ;
  g_cfg.init_terminal = CFG_DEFAULT_INIT_TERMINAL;
  g_cfg.init_debug_level = CFG_DEFAULT_INIT_DEBUG_LEVEL;
  g_cfg.wdt_use = CFG_DEFAULT_WDT_USE;
  g_cfg.motors = CFG_DEFAULT_MOTORS;
  g_cfg.servos = CFG_DEFAULT_SERVOS;
  g_cfg.name[0] = 0;
  strncpy(g_cfg.unit_name, CFG_DEFAULT_UNIT_NAME, sizeof(g_cfg.unit_name) - 1);
  g_cfg.unit_name[sizeof(g_cfg.unit_name) - 1] = 0;
  g_cfg.drv_step_active = CFG_DEFAULT_DRV_STEP_ACTIVE;
  g_cfg.drv_dir_active = CFG_DEFAULT_DRV_DIR_ACTIVE;
  g_cfg.drv_enable_active = CFG_DEFAULT_DRV_ENABLE_ACTIVE;
  g_cfg.drv_error_active = CFG_DEFAULT_DRV_ERROR_ACTIVE;
  g_cfg.sw_limit_l_active = CFG_DEFAULT_SW_LIMIT_L_ACTIVE;
  g_cfg.sw_limit_r_active = CFG_DEFAULT_SW_LIMIT_R_ACTIVE;
  g_cfg.sw_limit_l_use = CFG_DEFAULT_SW_LIMIT_L_USE;
  g_cfg.sw_limit_r_use = CFG_DEFAULT_SW_LIMIT_R_USE;
  g_cfg.buzzer_use = CFG_DEFAULT_BUZZER_USE;
  for (int e = 0; e < 4; ++e) {
    g_cfg.ext_active[e] = CFG_DEFAULT_EXT_ACTIVE;
  }
  g_cfg.home_mode = CFG_DEFAULT_HOME_MODE;
  g_cfg.home_move_out_mm = CFG_DEFAULT_HOME_MOVE_OUT_MM;
  g_cfg.home_speed_mm_s = CFG_DEFAULT_HOME_SPEED_MM_S;
  g_cfg.home_accel_mm_s2 = CFG_DEFAULT_HOME_ACCEL_MM_S2;
  init_axis2_defaults();
  init_axis3_defaults();
  g_cfg.ramp_start_hz = CFG_DEFAULT_RAMP_START_HZ;
  g_cfg.stop_approach_hz = CFG_DEFAULT_STOP_APPROACH_HZ;
  g_cfg.dir_change_pause_s = CFG_DEFAULT_DIR_CHANGE_PAUSE_S;
  g_cfg.path_buffer_size = CFG_DEFAULT_PATH_BUFFER_SIZE;
  if (g_cfg.path_buffer_size > PATH_BUFFER_MAX) {
    g_cfg.path_buffer_size = PATH_BUFFER_MAX;
  }
  g_cfg.init_path_slice_us = CFG_DEFAULT_INIT_PATH_SLICE_US;
  clamp_cfg_counts();
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

bool config_slider_min_enabled(void) { return !isnan(g_cfg.motor_min[0]); }
bool config_slider_max_enabled(void) { return !isnan(g_cfg.motor_max[0]); }

int config_motor_count(void) {
  clamp_cfg_counts();
  return g_cfg.motors;
}

int config_servo_count(void) {
  clamp_cfg_counts();
  return g_cfg.servos;
}

int config_axis_count(void) { return config_motor_count() + config_servo_count(); }

bool config_axis2_enabled(void) { return config_motor_count() >= 2; }

bool config_axis3_enabled(void) { return config_motor_count() >= 3; }

int config_path_per_axis_max(void) {
  int n = config_axis_count();
  if (n < 1) {
    n = 1;
  }
  return PATH_POOL_SAMPLES / n;
}

bool config_ext_available(int index0) {
  if (index0 < 0 || index0 >= PIN_EXT_COUNT) {
    return false;
  }
#if PIN_SERVO3_STEALS_EXT4
  if (index0 == 3 && config_servo_count() >= 3) {
    return false;
  }
#endif
  return true;
}

float config_channel_min(int ch) {
  int nm = g_cfg.motors;
  if (nm < 1) {
    nm = 1;
  }
  if (ch < 0 || ch >= MC_CH_MAX) {
    return NAN;
  }
  if (ch < nm) {
    return g_cfg.motor_min[ch];
  }
  int s = ch - nm;
  if (s < 0 || s >= SERVO_MAX) {
    return NAN;
  }
  return g_cfg.servo_min[s];
}

float config_channel_max(int ch) {
  int nm = g_cfg.motors;
  if (nm < 1) {
    nm = 1;
  }
  if (ch < 0 || ch >= MC_CH_MAX) {
    return NAN;
  }
  if (ch < nm) {
    return g_cfg.motor_max[ch];
  }
  int s = ch - nm;
  if (s < 0 || s >= SERVO_MAX) {
    return NAN;
  }
  return g_cfg.servo_max[s];
}

static bool config_set_channel_min(int ch, float v) {
  int nm = config_motor_count();
  if (ch < 0 || ch >= config_axis_count()) {
    return false;
  }
  if (ch < nm) {
    g_cfg.motor_min[ch] = v;
  } else {
    g_cfg.servo_min[ch - nm] = v;
  }
  session_clamp_window_to_envelope();
  return true;
}

static bool config_set_channel_max(int ch, float v) {
  int nm = config_motor_count();
  if (ch < 0 || ch >= config_axis_count()) {
    return false;
  }
  if (ch < nm) {
    g_cfg.motor_max[ch] = v;
  } else {
    g_cfg.servo_max[ch - nm] = v;
  }
  session_clamp_window_to_envelope();
  return true;
}

static float cfg_slider_min(int axis) { return config_channel_min(axis); }

static float cfg_slider_max(int axis) { return config_channel_max(axis); }

static float *session_soft_left(int axis) { return &g_session.soft_left[axis]; }

static float *session_soft_right(int axis) { return &g_session.soft_right[axis]; }

static bool pos_in_envelope(int axis, float pos) {
  float mn = cfg_slider_min(axis);
  float mx = cfg_slider_max(axis);
  if (!isnan(mn) && pos < mn - 1e-4f) {
    return false;
  }
  if (!isnan(mx) && pos > mx + 1e-4f) {
    return false;
  }
  return true;
}

float session_effective_left(int axis) {
  float soft = *session_soft_left(axis);
  if (!isnan(soft)) {
    return soft;
  }
  return cfg_slider_min(axis);
}

float session_effective_right(int axis) {
  float soft = *session_soft_right(axis);
  if (!isnan(soft)) {
    return soft;
  }
  return cfg_slider_max(axis);
}

static bool eff_left_le_right(int axis, float new_left_or_nan, bool setting_left) {
  float left = setting_left ? (isnan(new_left_or_nan) ? cfg_slider_min(axis) : new_left_or_nan)
                            : session_effective_left(axis);
  float right = session_effective_right(axis);
  if (isnan(left) || isnan(right)) {
    return true;
  }
  return left <= right + 1e-4f;
}

static bool eff_right_ge_left(int axis, float new_right_or_nan, bool setting_right) {
  float right =
      setting_right ? (isnan(new_right_or_nan) ? cfg_slider_max(axis) : new_right_or_nan)
                    : session_effective_right(axis);
  float left = session_effective_left(axis);
  if (isnan(left) || isnan(right)) {
    return true;
  }
  return left <= right + 1e-4f;
}

static bool session_set_window_one(int axis, bool set, float mm, bool left) {
  if (!set || axis >= config_axis_count()) {
    return true;
  }
  if (!isnan(mm) && !pos_in_envelope(axis, mm)) {
    return false;
  }
  if (left) {
    if (!eff_left_le_right(axis, mm, true)) {
      return false;
    }
  } else if (!eff_right_ge_left(axis, mm, true)) {
    return false;
  }
  return true;
}

bool session_set_window_left(const bool set[MC_CH_MAX], const float mm[MC_CH_MAX]) {
  int n = config_axis_count();
  for (int i = 0; i < n; ++i) {
    if (!session_set_window_one(i, set[i], mm[i], true)) {
      return false;
    }
  }
  for (int i = 0; i < n; ++i) {
    if (set[i]) {
      g_session.soft_left[i] = mm[i];
    }
  }
  return true;
}

bool session_set_window_right(const bool set[MC_CH_MAX], const float mm[MC_CH_MAX]) {
  int n = config_axis_count();
  for (int i = 0; i < n; ++i) {
    if (!session_set_window_one(i, set[i], mm[i], false)) {
      return false;
    }
  }
  for (int i = 0; i < n; ++i) {
    if (set[i]) {
      g_session.soft_right[i] = mm[i];
    }
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

static bool set_servo_pulse(const char *value, int *field, int other, bool field_is_min) {
  int i;
  if (!parse_int(value, &i) || i < CFG_SERVO_PULSE_US_LO || i > CFG_SERVO_PULSE_US_HI) {
    return false;
  }
  if (field_is_min) {
    if (i >= other) {
      return false;
    }
  } else if (i <= other) {
    return false;
  }
  *field = i;
  return true;
}

/* kind_N_field with N in 1..3, e.g. MOTOR_2_min, SERVO_1_max_speed. */
static bool match_kind_n_field(const char *key, const char *kind, const char *field, int *idx0) {
  size_t kn = strlen(kind);
  size_t i = 0;
  for (; i < kn; ++i) {
    char ca = key[i];
    char cb = kind[i];
    if (ca >= 'A' && ca <= 'Z') {
      ca = (char)(ca - 'A' + 'a');
    }
    if (cb >= 'A' && cb <= 'Z') {
      cb = (char)(cb - 'A' + 'a');
    }
    if (!key[i] || ca != cb) {
      return false;
    }
  }
  if (key[i] != '_') {
    return false;
  }
  ++i;
  if (key[i] < '1' || key[i] > '3') {
    return false;
  }
  int n = key[i] - '1';
  ++i;
  if (key[i] != '_') {
    return false;
  }
  ++i;
  if (icmp(key + i, field) != 0) {
    return false;
  }
  *idx0 = n;
  return true;
}

/* axis_min_N / axis_max_N / soft_min_N / soft_max_N, N=1..6 */
static bool match_synth_env(const char *key, bool *is_min, int *idx0) {
  const char *p = key;
  if (key_is(p, "axis_min_1", nullptr) || false) {
    /* fall through to generic */
  }
  bool axis = (p[0] == 'a' || p[0] == 'A') && (p[1] == 'x' || p[1] == 'X') &&
              (p[2] == 'i' || p[2] == 'I') && (p[3] == 's' || p[3] == 'S') && p[4] == '_';
  bool soft = (p[0] == 's' || p[0] == 'S') && (p[1] == 'o' || p[1] == 'O') &&
              (p[2] == 'f' || p[2] == 'F') && (p[3] == 't' || p[3] == 'T') && p[4] == '_';
  if (!axis && !soft) {
    return false;
  }
  p += 5;
  if ((p[0] == 'm' || p[0] == 'M') && (p[1] == 'i' || p[1] == 'I') &&
      (p[2] == 'n' || p[2] == 'N') && p[3] == '_') {
    *is_min = true;
    p += 4;
  } else if ((p[0] == 'm' || p[0] == 'M') && (p[1] == 'a' || p[1] == 'A') &&
             (p[2] == 'x' || p[2] == 'X') && p[3] == '_') {
    *is_min = false;
    p += 4;
  } else {
    return false;
  }
  if (p[0] < '1' || p[0] > '6' || p[1] != 0) {
    return false;
  }
  *idx0 = p[0] - '1';
  return true;
}

static bool set_env_none_or_float(const char *value, float *dst) {
  if (is_none_token(value)) {
    *dst = NAN;
    session_clamp_window_to_envelope();
    return true;
  }
  float f;
  if (!parse_float(value, &f)) {
    return false;
  }
  *dst = f;
  session_clamp_window_to_envelope();
  return true;
}

static bool counts_change_ok(void) {
  if (motion_path_is_active()) {
    return false;
  }
  if (motion_path_count() > 0) {
    if (!motion_path_clear()) {
      return false;
    }
  }
  return true;
}

static bool fmt_env(char *out, size_t out_len, float v) {
  if (isnan(v)) {
    snprintf(out, out_len, "none");
  } else {
    snprintf(out, out_len, "%.3g", (double)v);
  }
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
    g_session.decel_mm_s2 = f;
    return true;
  }
  if (icmp(key, "max_speed_1") == 0) {
    if (!parse_float(value, &f) || f <= 0.0f) {
      return false;
    }
    g_cfg.max_speed_mm_s = f;
    return true;
  }
  if (icmp(key, "max_accel_1") == 0) {
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
  if (key_is(key, "steps_per_unit_1", "steps_per_mm_1")) {
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
  {
    int idx = 0;
    if (match_kind_n_field(key, "MOTOR", "min", &idx)) {
      return set_env_none_or_float(value, &g_cfg.motor_min[idx]);
    }
    if (match_kind_n_field(key, "MOTOR", "max", &idx)) {
      return set_env_none_or_float(value, &g_cfg.motor_max[idx]);
    }
    if (match_kind_n_field(key, "SERVO", "min_pulse", &idx)) {
      return set_servo_pulse(value, &g_cfg.servo_min_pulse[idx], g_cfg.servo_max_pulse[idx],
                             true);
    }
    if (match_kind_n_field(key, "SERVO", "max_pulse", &idx)) {
      return set_servo_pulse(value, &g_cfg.servo_max_pulse[idx], g_cfg.servo_min_pulse[idx],
                             false);
    }
    if (match_kind_n_field(key, "SERVO", "swap", &idx)) {
      return set_01(value, &g_cfg.servo_swap[idx]);
    }
    if (match_kind_n_field(key, "SERVO", "min", &idx)) {
      return set_env_none_or_float(value, &g_cfg.servo_min[idx]);
    }
    if (match_kind_n_field(key, "SERVO", "max", &idx)) {
      return set_env_none_or_float(value, &g_cfg.servo_max[idx]);
    }
    if (match_kind_n_field(key, "SERVO", "max_speed", &idx)) {
      if (!parse_float(value, &f) || f <= 0.0f) {
        return false;
      }
      g_cfg.servo_max_speed[idx] = f;
      return true;
    }
    if (match_kind_n_field(key, "SERVO", "max_accel", &idx)) {
      if (!parse_float(value, &f) || f <= 0.0f) {
        return false;
      }
      g_cfg.servo_max_accel[idx] = f;
      return true;
    }
    if (match_kind_n_field(key, "SERVO", "active", &idx)) {
      return set_01(value, &g_cfg.servo_active[idx]);
    }
  }
  {
    bool is_min = false;
    int idx = 0;
    if (match_synth_env(key, &is_min, &idx)) {
      if (is_none_token(value)) {
        return is_min ? config_set_channel_min(idx, NAN) : config_set_channel_max(idx, NAN);
      }
      if (!parse_float(value, &f)) {
        return false;
      }
      return is_min ? config_set_channel_min(idx, f) : config_set_channel_max(idx, f);
    }
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
  if (icmp(key, "axis") == 0) {
    return false; /* CS axis rejected; use motors/servos. CG axis still works. */
  }
  if ((key[0] == 's' || key[0] == 'S') && (key[1] == 'l' || key[1] == 'L') &&
      (key[2] == 'i' || key[2] == 'I') && (key[3] == 'd' || key[3] == 'D') &&
      (key[4] == 'e' || key[4] == 'E') && (key[5] == 'r' || key[5] == 'R') && key[6] == '_') {
    return false; /* slider_min_* / slider_max_* rejected */
  }
  if (icmp(key, "motors") == 0) {
    if (!parse_int(value, &i) || i < 1 || i > 3) {
      return false;
    }
    if (i > axis_board_max()) {
      return false;
    }
    if (i != g_cfg.motors) {
      if (!counts_change_ok()) {
        return false;
      }
      g_cfg.motors = i;
      clamp_cfg_counts();
      session_reset_left();
      session_reset_right();
      return true;
    }
    g_cfg.motors = i;
    clamp_cfg_counts();
    return true;
  }
  if (icmp(key, "servos") == 0) {
    if (!parse_int(value, &i) || i < 0 || i > SERVO_MAX) {
      return false;
    }
    if (i != g_cfg.servos) {
      if (!counts_change_ok()) {
        return false;
      }
      g_cfg.servos = i;
      clamp_cfg_counts();
      session_reset_left();
      session_reset_right();
      return true;
    }
    g_cfg.servos = i;
    clamp_cfg_counts();
    return true;
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
  if (icmp(key, "DRV_STEP_1_active") == 0) {
    return set_01(value, &g_cfg.drv_step_active);
  }
  if (icmp(key, "DRV_DIR_1_active") == 0) {
    return set_01(value, &g_cfg.drv_dir_active);
  }
  if (icmp(key, "DRV_ENABLE_active") == 0) {
    return set_01(value, &g_cfg.drv_enable_active);
  }
  if (icmp(key, "DRV_ERROR_1_active") == 0) {
    return set_01(value, &g_cfg.drv_error_active);
  }
  if (icmp(key, "SW_LIMIT_L_1_active") == 0) {
    return set_01(value, &g_cfg.sw_limit_l_active);
  }
  if (icmp(key, "SW_LIMIT_R_1_active") == 0) {
    return set_01(value, &g_cfg.sw_limit_r_active);
  }
  if (icmp(key, "SW_LIMIT_L_1_use") == 0) {
    return set_01(value, &g_cfg.sw_limit_l_use);
  }
  if (icmp(key, "SW_LIMIT_R_1_use") == 0) {
    return set_01(value, &g_cfg.sw_limit_r_use);
  }
  if (icmp(key, "BUZZER_use") == 0) {
    return set_01(value, &g_cfg.buzzer_use);
  }
  if (icmp(key, "DRV_STEP_2_active") == 0) {
    return set_01(value, &g_cfg.drv_step_active_2);
  }
  if (icmp(key, "DRV_DIR_2_active") == 0) {
    return set_01(value, &g_cfg.drv_dir_active_2);
  }
  if (icmp(key, "DRV_ERROR_2_active") == 0) {
    return set_01(value, &g_cfg.drv_error_active_2);
  }
  if (icmp(key, "SW_LIMIT_L_2_active") == 0) {
    return set_01(value, &g_cfg.sw_limit_l_active_2);
  }
  if (icmp(key, "SW_LIMIT_R_2_active") == 0) {
    return set_01(value, &g_cfg.sw_limit_r_active_2);
  }
  if (icmp(key, "SW_LIMIT_L_2_use") == 0) {
    return set_01(value, &g_cfg.sw_limit_l_use_2);
  }
  if (icmp(key, "SW_LIMIT_R_2_use") == 0) {
    return set_01(value, &g_cfg.sw_limit_r_use_2);
  }
  /* EXT_1_active … EXT_4_active */
  if ((key[0] == 'E' || key[0] == 'e') && (key[1] == 'X' || key[1] == 'x') &&
      (key[2] == 'T' || key[2] == 't') && key[3] == '_' && key[4] >= '1' && key[4] <= '4' &&
      key[5] == '_' && icmp(key + 6, "active") == 0) {
    return set_01(value, &g_cfg.ext_active[key[4] - '1']);
  }
  if (icmp(key, "home_mode_1") == 0) {
    if (!parse_int(value, &i) || i < 0 || i > 4) {
      return false;
    }
    g_cfg.home_mode = i;
    return true;
  }
  if (icmp(key, "home_move_out_1") == 0) {
    if (!parse_float(value, &f) || f < 0.0f) {
      return false;
    }
    g_cfg.home_move_out_mm = f;
    return true;
  }
  if (icmp(key, "home_speed_1") == 0) {
    if (!parse_float(value, &f) || f <= 0.0f) {
      return false;
    }
    g_cfg.home_speed_mm_s = f;
    return true;
  }
  if (icmp(key, "home_accel_1") == 0) {
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
  if (icmp(key, "home_move_out_2") == 0) {
    if (!parse_float(value, &f) || f < 0.0f) {
      return false;
    }
    g_cfg.home_move_out_mm_2 = f;
    return true;
  }
  if (icmp(key, "home_speed_2") == 0) {
    if (!parse_float(value, &f) || f <= 0.0f) {
      return false;
    }
    g_cfg.home_speed_mm_s_2 = f;
    return true;
  }
  if (icmp(key, "home_accel_2") == 0) {
    if (!parse_float(value, &f) || f <= 0.0f) {
      return false;
    }
    g_cfg.home_accel_mm_s2_2 = f;
    return true;
  }
  if (icmp(key, "max_speed_3") == 0) {
    if (!parse_float(value, &f) || f <= 0.0f) {
      return false;
    }
    g_cfg.max_speed_mm_s_3 = f;
    return true;
  }
  if (icmp(key, "max_accel_3") == 0) {
    if (!parse_float(value, &f) || f <= 0.0f) {
      return false;
    }
    g_cfg.max_accel_mm_s2_3 = f;
    return true;
  }
  if (key_is(key, "steps_per_unit_3", "steps_per_mm_3")) {
    if (!parse_float(value, &f) || f <= 0.0f) {
      return false;
    }
    g_cfg.steps_per_unit_3 = f;
    return true;
  }
  if (icmp(key, "DRV_DIR_3_active") == 0) {
    return set_01(value, &g_cfg.drv_dir_active_3);
  }
  if (icmp(key, "DRV_ERROR_3_active") == 0) {
    return set_01(value, &g_cfg.drv_error_active_3);
  }
  if (icmp(key, "SW_LIMIT_L_3_active") == 0) {
    return set_01(value, &g_cfg.sw_limit_l_active_3);
  }
  if (icmp(key, "SW_LIMIT_R_3_active") == 0) {
    return set_01(value, &g_cfg.sw_limit_r_active_3);
  }
  if (icmp(key, "SW_LIMIT_L_3_use") == 0) {
    return set_01(value, &g_cfg.sw_limit_l_use_3);
  }
  if (icmp(key, "SW_LIMIT_R_3_use") == 0) {
    return set_01(value, &g_cfg.sw_limit_r_use_3);
  }
  if (icmp(key, "home_mode_3") == 0) {
    if (!parse_int(value, &i) || i < 0 || i > 4) {
      return false;
    }
    g_cfg.home_mode_3 = i;
    return true;
  }
  if (icmp(key, "home_move_out_3") == 0) {
    if (!parse_float(value, &f) || f < 0.0f) {
      return false;
    }
    g_cfg.home_move_out_mm_3 = f;
    return true;
  }
  if (icmp(key, "home_speed_3") == 0) {
    if (!parse_float(value, &f) || f <= 0.0f) {
      return false;
    }
    g_cfg.home_speed_mm_s_3 = f;
    return true;
  }
  if (icmp(key, "home_accel_3") == 0) {
    if (!parse_float(value, &f) || f <= 0.0f) {
      return false;
    }
    g_cfg.home_accel_mm_s2_3 = f;
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
    int cap = config_path_per_axis_max();
    if (!parse_int(value, &i) || i < 1 || i > cap) {
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
  if (icmp(key, "max_speed_1") == 0) {
    snprintf(out, out_len, "%.3g", (double)c->max_speed_mm_s);
    return true;
  }
  if (icmp(key, "max_accel_1") == 0) {
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
  if (key_is(key, "steps_per_unit_1", "steps_per_mm_1")) {
    snprintf(out, out_len, "%.6g", (double)c->steps_per_unit);
    return true;
  }
  if (key_is(key, "steps_per_unit_2", "steps_per_mm_2")) {
    snprintf(out, out_len, "%.6g", (double)c->steps_per_unit_2);
    return true;
  }
  {
    int idx = 0;
    if (match_kind_n_field(key, "MOTOR", "min", &idx)) {
      return fmt_env(out, out_len, c->motor_min[idx]);
    }
    if (match_kind_n_field(key, "MOTOR", "max", &idx)) {
      return fmt_env(out, out_len, c->motor_max[idx]);
    }
    if (match_kind_n_field(key, "SERVO", "min_pulse", &idx)) {
      snprintf(out, out_len, "%d", c->servo_min_pulse[idx]);
      return true;
    }
    if (match_kind_n_field(key, "SERVO", "max_pulse", &idx)) {
      snprintf(out, out_len, "%d", c->servo_max_pulse[idx]);
      return true;
    }
    if (match_kind_n_field(key, "SERVO", "swap", &idx)) {
      snprintf(out, out_len, "%d", c->servo_swap[idx]);
      return true;
    }
    if (match_kind_n_field(key, "SERVO", "min", &idx)) {
      return fmt_env(out, out_len, c->servo_min[idx]);
    }
    if (match_kind_n_field(key, "SERVO", "max", &idx)) {
      return fmt_env(out, out_len, c->servo_max[idx]);
    }
    if (match_kind_n_field(key, "SERVO", "max_speed", &idx)) {
      snprintf(out, out_len, "%.3g", (double)c->servo_max_speed[idx]);
      return true;
    }
    if (match_kind_n_field(key, "SERVO", "max_accel", &idx)) {
      snprintf(out, out_len, "%.3g", (double)c->servo_max_accel[idx]);
      return true;
    }
    if (match_kind_n_field(key, "SERVO", "active", &idx)) {
      snprintf(out, out_len, "%d", c->servo_active[idx]);
      return true;
    }
  }
  {
    bool is_min = false;
    int idx = 0;
    if (match_synth_env(key, &is_min, &idx)) {
      return fmt_env(out, out_len, is_min ? config_channel_min(idx) : config_channel_max(idx));
    }
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
  if (icmp(key, "axis") == 0) {
    snprintf(out, out_len, "%d", config_axis_count());
    return true;
  }
  if (icmp(key, "motors") == 0) {
    snprintf(out, out_len, "%d", config_motor_count());
    return true;
  }
  if (icmp(key, "servos") == 0) {
    snprintf(out, out_len, "%d", config_servo_count());
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
  if (icmp(key, "DRV_STEP_1_active") == 0) {
    snprintf(out, out_len, "%d", c->drv_step_active);
    return true;
  }
  if (icmp(key, "DRV_DIR_1_active") == 0) {
    snprintf(out, out_len, "%d", c->drv_dir_active);
    return true;
  }
  if (icmp(key, "DRV_ENABLE_active") == 0) {
    snprintf(out, out_len, "%d", c->drv_enable_active);
    return true;
  }
  if (icmp(key, "DRV_ERROR_1_active") == 0) {
    snprintf(out, out_len, "%d", c->drv_error_active);
    return true;
  }
  if (icmp(key, "SW_LIMIT_L_1_active") == 0) {
    snprintf(out, out_len, "%d", c->sw_limit_l_active);
    return true;
  }
  if (icmp(key, "SW_LIMIT_R_1_active") == 0) {
    snprintf(out, out_len, "%d", c->sw_limit_r_active);
    return true;
  }
  if (icmp(key, "SW_LIMIT_L_1_use") == 0) {
    snprintf(out, out_len, "%d", c->sw_limit_l_use);
    return true;
  }
  if (icmp(key, "SW_LIMIT_R_1_use") == 0) {
    snprintf(out, out_len, "%d", c->sw_limit_r_use);
    return true;
  }
  if (icmp(key, "BUZZER_use") == 0) {
    snprintf(out, out_len, "%d", c->buzzer_use);
    return true;
  }
  if (icmp(key, "DRV_STEP_2_active") == 0) {
    snprintf(out, out_len, "%d", c->drv_step_active_2);
    return true;
  }
  if (icmp(key, "DRV_DIR_2_active") == 0) {
    snprintf(out, out_len, "%d", c->drv_dir_active_2);
    return true;
  }
  if (icmp(key, "DRV_ERROR_2_active") == 0) {
    snprintf(out, out_len, "%d", c->drv_error_active_2);
    return true;
  }
  if (icmp(key, "SW_LIMIT_L_2_active") == 0) {
    snprintf(out, out_len, "%d", c->sw_limit_l_active_2);
    return true;
  }
  if (icmp(key, "SW_LIMIT_R_2_active") == 0) {
    snprintf(out, out_len, "%d", c->sw_limit_r_active_2);
    return true;
  }
  if (icmp(key, "SW_LIMIT_L_2_use") == 0) {
    snprintf(out, out_len, "%d", c->sw_limit_l_use_2);
    return true;
  }
  if (icmp(key, "SW_LIMIT_R_2_use") == 0) {
    snprintf(out, out_len, "%d", c->sw_limit_r_use_2);
    return true;
  }
  if ((key[0] == 'E' || key[0] == 'e') && (key[1] == 'X' || key[1] == 'x') &&
      (key[2] == 'T' || key[2] == 't') && key[3] == '_' && key[4] >= '1' && key[4] <= '4' &&
      key[5] == '_' && icmp(key + 6, "active") == 0) {
    snprintf(out, out_len, "%d", c->ext_active[key[4] - '1']);
    return true;
  }
  if (icmp(key, "home_mode_1") == 0) {
    snprintf(out, out_len, "%d", c->home_mode);
    return true;
  }
  if (icmp(key, "home_move_out_1") == 0) {
    snprintf(out, out_len, "%.3g", (double)c->home_move_out_mm);
    return true;
  }
  if (icmp(key, "home_speed_1") == 0) {
    snprintf(out, out_len, "%.3g", (double)c->home_speed_mm_s);
    return true;
  }
  if (icmp(key, "home_accel_1") == 0) {
    snprintf(out, out_len, "%.3g", (double)c->home_accel_mm_s2);
    return true;
  }
  if (icmp(key, "home_mode_2") == 0) {
    snprintf(out, out_len, "%d", c->home_mode_2);
    return true;
  }
  if (icmp(key, "home_move_out_2") == 0) {
    snprintf(out, out_len, "%.3g", (double)c->home_move_out_mm_2);
    return true;
  }
  if (icmp(key, "home_speed_2") == 0) {
    snprintf(out, out_len, "%.3g", (double)c->home_speed_mm_s_2);
    return true;
  }
  if (icmp(key, "home_accel_2") == 0) {
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
  if (icmp(key, "max_speed_3") == 0) {
    snprintf(out, out_len, "%.3g", (double)c->max_speed_mm_s_3);
    return true;
  }
  if (icmp(key, "max_accel_3") == 0) {
    snprintf(out, out_len, "%.3g", (double)c->max_accel_mm_s2_3);
    return true;
  }
  if (key_is(key, "steps_per_unit_3", "steps_per_mm_3")) {
    snprintf(out, out_len, "%.6g", (double)c->steps_per_unit_3);
    return true;
  }
  if (icmp(key, "DRV_DIR_3_active") == 0) {
    snprintf(out, out_len, "%d", c->drv_dir_active_3);
    return true;
  }
  if (icmp(key, "DRV_ERROR_3_active") == 0) {
    snprintf(out, out_len, "%d", c->drv_error_active_3);
    return true;
  }
  if (icmp(key, "SW_LIMIT_L_3_active") == 0) {
    snprintf(out, out_len, "%d", c->sw_limit_l_active_3);
    return true;
  }
  if (icmp(key, "SW_LIMIT_R_3_active") == 0) {
    snprintf(out, out_len, "%d", c->sw_limit_r_active_3);
    return true;
  }
  if (icmp(key, "SW_LIMIT_L_3_use") == 0) {
    snprintf(out, out_len, "%d", c->sw_limit_l_use_3);
    return true;
  }
  if (icmp(key, "SW_LIMIT_R_3_use") == 0) {
    snprintf(out, out_len, "%d", c->sw_limit_r_use_3);
    return true;
  }
  if (icmp(key, "home_mode_3") == 0) {
    snprintf(out, out_len, "%d", c->home_mode_3);
    return true;
  }
  if (icmp(key, "home_move_out_3") == 0) {
    snprintf(out, out_len, "%.3g", (double)c->home_move_out_mm_3);
    return true;
  }
  if (icmp(key, "home_speed_3") == 0) {
    snprintf(out, out_len, "%.3g", (double)c->home_speed_mm_s_3);
    return true;
  }
  if (icmp(key, "home_accel_3") == 0) {
    snprintf(out, out_len, "%.3g", (double)c->home_accel_mm_s2_3);
    return true;
  }
  return false;
}

void config_foreach(config_foreach_fn fn, void *ctx) {
  static const char *keys[] = {
      "max_speed_1",
      "max_accel_1",
      "max_speed_2",
      "max_accel_2",
      "init_speed",
      "init_accel",
      "steps_per_unit_1",
      "unit_name",
      "MOTOR_1_min",
      "MOTOR_1_max",
      "init_verbose",
      "verbose_rate_hz",
      "init_terminal",
      "init_debug_level",
      "WDT_use",
      "motors",
      "servos",
      "axis",
      "name",
      "DRV_STEP_1_active",
      "DRV_DIR_1_active",
      "DRV_ENABLE_active",
      "DRV_ERROR_1_active",
      "SW_LIMIT_L_1_active",
      "SW_LIMIT_R_1_active",
      "SW_LIMIT_L_1_use",
      "SW_LIMIT_R_1_use",
      "BUZZER_use",
      "EXT_1_active",
      "EXT_2_active",
      "EXT_3_active",
      "EXT_4_active",
      "home_mode_1",
      "home_move_out_1",
      "home_speed_1",
      "home_accel_1",
      "steps_per_unit_2",
      "MOTOR_2_min",
      "MOTOR_2_max",
      "DRV_STEP_2_active",
      "DRV_DIR_2_active",
      "DRV_ERROR_2_active",
      "SW_LIMIT_L_2_active",
      "SW_LIMIT_R_2_active",
      "SW_LIMIT_L_2_use",
      "SW_LIMIT_R_2_use",
      "home_mode_2",
      "home_move_out_2",
      "home_speed_2",
      "home_accel_2",
      "max_speed_3",
      "max_accel_3",
      "steps_per_unit_3",
      "MOTOR_3_min",
      "MOTOR_3_max",
      "DRV_DIR_3_active",
      "DRV_ERROR_3_active",
      "SW_LIMIT_L_3_active",
      "SW_LIMIT_R_3_active",
      "SW_LIMIT_L_3_use",
      "SW_LIMIT_R_3_use",
      "home_mode_3",
      "home_move_out_3",
      "home_speed_3",
      "home_accel_3",
      "SERVO_1_min",
      "SERVO_1_max",
      "SERVO_1_max_speed",
      "SERVO_1_max_accel",
      "SERVO_1_active",
      "SERVO_1_min_pulse",
      "SERVO_1_max_pulse",
      "SERVO_1_swap",
      "SERVO_2_min",
      "SERVO_2_max",
      "SERVO_2_max_speed",
      "SERVO_2_max_accel",
      "SERVO_2_active",
      "SERVO_2_min_pulse",
      "SERVO_2_max_pulse",
      "SERVO_2_swap",
      "SERVO_3_min",
      "SERVO_3_max",
      "SERVO_3_max_speed",
      "SERVO_3_max_accel",
      "SERVO_3_active",
      "SERVO_3_min_pulse",
      "SERVO_3_max_pulse",
      "SERVO_3_swap",
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
  int nch = config_axis_count();
  for (int i = 1; i <= nch && i <= MC_CH_MAX; ++i) {
    char k[16];
    snprintf(k, sizeof(k), "axis_min_%d", i);
    if (config_get_key(k, val, sizeof(val))) {
      fn(k, val, ctx);
    }
    snprintf(k, sizeof(k), "axis_max_%d", i);
    if (config_get_key(k, val, sizeof(val))) {
      fn(k, val, ctx);
    }
  }
}
