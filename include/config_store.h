#pragma once

#include "config_defaults.h"

#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  float init_speed_mm_s;
  float init_accel_mm_s2;
  float max_speed_mm_s;
  float max_accel_mm_s2;
  float steps_per_unit; /* steps per user unit (mm, deg, …) */
  float slider_min_mm; /* NAN = disabled; value in user units */
  float slider_max_mm; /* NAN = disabled */
  int init_verbose;
  int verbose_rate_hz; /* verbose `#…` push rate (1..200; proto task polls at 200 Hz) */
  int init_terminal; /* Terminal Mode (local echo) */
  int init_debug_level;
  int wdt_use; /* 1 = arm RP2040 WDT (2 s) from protocol heartbeat */
  int axis2_use; /* 1 = enable 2nd STEP/DIR axis (Pico only) */
  char name[CFG_NAME_MAX]; /* optional device name for welcome banner; empty = omit */
  char unit_name[CFG_UNIT_NAME_MAX]; /* UIC label for user units; default "mm" */

  /* Pin active level: 0 = low-active / asserted at 0, 1 = high-active */
  int drv_step_active;
  int drv_dir_active; /* 1 = DIR high means +mm */
  int drv_en_active;
  int sw_home_active;
  int sw_home_use; /* 1 = poll PIN_SW_HOME for MH only (no hard-limit trip) */
  int drv_error_active;
  int sw_limit_l_active;
  int sw_limit_r_active;
  int sw_limit_l_use; /* 1 = poll PIN_SW_LIMIT_L as hard limit */
  int sw_limit_r_use;
  int ext_active[4]; /* EXT_0..3_active: 0=low-active, 1=high-active */

  /* Homing: 0=off, 1=SW_HOME left, 2=SW_HOME right, 3=LIMIT_L, 4=LIMIT_R */
  int home_mode;
  float home_move_out_mm;
  float home_speed_mm_s;
  float home_accel_mm_s2;

  /* Axis 2 mirrors (used when axis2_use=1 and PIN_AXIS2_SUPPORTED) */
  float steps_per_unit_2;
  float slider_min_mm_2;
  float slider_max_mm_2;
  int drv_step_active_2;
  int drv_dir_active_2;
  int drv_en_active_2;
  int drv_error_active_2;
  int sw_home_active_2;
  int sw_home_use_2;
  int sw_limit_l_active_2;
  int sw_limit_r_active_2;
  int sw_limit_l_use_2;
  int sw_limit_r_use_2;
  int home_mode_2;
  float home_move_out_mm_2;
  float home_speed_mm_s_2;
  float home_accel_mm_s2_2;
  float max_speed_mm_s_2;
  float max_accel_mm_s2_2;

  int ramp_start_hz;
  int stop_approach_hz;
  float dir_change_pause_s;

  int path_buffer_size;    /* PD sample capacity (1..PATH_BUFFER_MAX) */
  int init_path_slice_us;  /* default PS slice length (>=1000 us) */
} McConfig;

/* Live session values for S/G commands (not written to mc.ini by S-commands). */
typedef struct {
  float speed_mm_s;
  float accel_mm_s2;
  int terminal;
  int verbose;
  int path_slice_us;
  /* Working window (SL/SR); NAN = cleared (effective clip falls back to envelope). */
  float soft_left_mm;
  float soft_right_mm;
  float soft_left_mm_2;
  float soft_right_mm_2;
} McSession;

void config_init_defaults(void);
/** Force all McConfig fields to compile-time defaults and sync session. */
void config_reset_to_defaults(void);
McConfig *config_get(void);
McSession *session_get(void);

/* Copy all session fields from persistent config (boot / after load). */
void session_sync_from_config(void);

/* Reload one session field from config init. */
void session_reset_speed(void);
void session_reset_accel(void);
void session_reset_terminal(void);
void session_reset_verbose(void);
void session_reset_path_slice(void);
void session_reset_left(void);  /* both axes → slider_min / _2 */
void session_reset_right(void); /* both axes → slider_max / _2 */
/** If CS squeezed the envelope, pull session walls inward (never past each other). */
void session_clamp_window_to_envelope(void);
/**
 * Set session window left/right. setN=false → leave that axis unchanged;
 * setN=true + NAN → store None (effective clip falls back to envelope).
 * false if outside envelope or effective left>right.
 */
bool session_set_window_left(bool set0, float mm0, bool set1, float mm1);
bool session_set_window_right(bool set0, float mm0, bool set1, float mm1);
/** Effective working-window min/max (session, else envelope if set, else NAN). */
float session_effective_left(int axis);
float session_effective_right(int axis);

/* Returns true on success. */
bool config_set_key(const char *key, const char *value);
bool config_get_key(const char *key, char *out, size_t out_len);

/* Emit all keys via callback: void cb(const char *key, const char *value, void *ctx) */
typedef void (*config_foreach_fn)(const char *key, const char *value, void *ctx);
void config_foreach(config_foreach_fn fn, void *ctx);

bool config_slider_min_enabled(void);
bool config_slider_max_enabled(void);

/**
 * True only if axis2_use && PIN_AXIS2_SUPPORTED.
 * On unsupported boards, clears a stray axis2_use=1 and returns false.
 */
bool config_axis2_enabled(void);

/* True if gpio_level (0/1) matches the pin's active setting. */
bool config_pin_asserted(int gpio_level, int active);

#ifdef __cplusplus
}
#endif
