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
  float motor_min[MOTOR_MAX]; /* NAN = disabled; user units */
  float motor_max[MOTOR_MAX];
  float servo_min[SERVO_MAX];
  float servo_max[SERVO_MAX];
  float servo_max_speed[SERVO_MAX];
  float servo_max_accel[SERVO_MAX];
  int servo_active[SERVO_MAX]; /* 1 = high pulse, 0 = invert */
  int init_verbose;
  int verbose_rate_hz; /* verbose `#…` push rate (1..200; proto task polls at 200 Hz) */
  int init_terminal; /* Terminal Mode (local echo) */
  int init_debug_level;
  int wdt_use; /* 1 = arm RP2040 WDT (2 s) from protocol heartbeat */
  int motors; /* 1|2|3 live STEP/DIR axes (clamped to board support) */
  int servos; /* 0..3 RC servo PWM channels */
  char name[CFG_NAME_MAX]; /* optional device name for welcome banner; empty = omit */
  char unit_name[CFG_UNIT_NAME_MAX]; /* UIC label for user units; default "mm" */

  /* Pin active level: 0 = low-active / asserted at 0, 1 = high-active */
  int drv_step_active;
  int drv_dir_active; /* 1 = DIR high means +mm */
  int drv_enable_active;
  int drv_error_active;
  int sw_limit_l_active;
  int sw_limit_r_active;
  int sw_limit_l_use; /* 1 = poll PIN_SW_LIMIT_L_1 as hard limit */
  int sw_limit_r_use;
  int buzzer_use; /* 1 = drive PIN_BUZZER from Z (skipped if pin aliases PIN_LED) */
  int ext_active[4]; /* EXT_1..4_active: 0=low-active, 1=high-active */

  /* Homing: 0=off, 1=LIMIT_L, 2=LIMIT_R, 3=stall/DRV_ERROR left, 4=stall right */
  int home_mode;
  float home_move_out_mm;
  float home_speed_mm_s;
  float home_accel_mm_s2;

  /* Axis 2 mirrors (used when motors>=2 and PIN_AXIS2_SUPPORTED) */
  float steps_per_unit_2;
  int drv_step_active_2;
  int drv_dir_active_2;
  int drv_error_active_2;
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

  /* Axis 3 mirrors (used when motors>=3). No drv_step_active_3 — follows axis 2. */
  float steps_per_unit_3;
  int drv_dir_active_3;
  int drv_error_active_3;
  int sw_limit_l_active_3;
  int sw_limit_r_active_3;
  int sw_limit_l_use_3;
  int sw_limit_r_use_3;
  int home_mode_3;
  float home_move_out_mm_3;
  float home_speed_mm_s_3;
  float home_accel_mm_s2_3;
  float max_speed_mm_s_3;
  float max_accel_mm_s2_3;

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
  float soft_left[MC_CH_MAX];
  float soft_right[MC_CH_MAX];
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
void session_reset_left(void);  /* all live channels → envelope min */
void session_reset_right(void); /* all live channels → envelope max */
/** If CS squeezed the envelope, pull session walls inward (never past each other). */
void session_clamp_window_to_envelope(void);
/**
 * Set session window left/right. set[i]=false → leave that channel unchanged;
 * set[i]=true + NAN → store None (effective clip falls back to envelope).
 * false if outside envelope or effective left>right.
 */
bool session_set_window_left(const bool set[MC_CH_MAX], const float mm[MC_CH_MAX]);
bool session_set_window_right(const bool set[MC_CH_MAX], const float mm[MC_CH_MAX]);
/** Effective working-window min/max (session, else envelope if set, else NAN). Packed channel. */
float session_effective_left(int ch);
float session_effective_right(int ch);

/* Returns true on success. */
bool config_set_key(const char *key, const char *value);
bool config_get_key(const char *key, char *out, size_t out_len);

/* Emit all keys via callback: void cb(const char *key, const char *value, void *ctx) */
typedef void (*config_foreach_fn)(const char *key, const char *value, void *ctx);
void config_foreach(config_foreach_fn fn, void *ctx);

bool config_slider_min_enabled(void);
bool config_slider_max_enabled(void);

/** Live STEP/DIR count 1..3 after board clamp. */
int config_motor_count(void);
/** Live RC servo count 0..3. */
int config_servo_count(void);
/** Packed channel count: motors + servos (IA / CG axis). */
int config_axis_count(void);
/** True if motors >= 2 (and board supports axis 2). */
bool config_axis2_enabled(void);
/** True if motors >= 3 (and board supports axis 3). */
bool config_axis3_enabled(void);
/** Packed-channel envelope (motors then servos). */
float config_channel_min(int ch);
float config_channel_max(int ch);
/** True if EXT index 0..3 is available (false for EXT_4 on Pico when servos>=3). */
bool config_ext_available(int index0);
/** Per-channel path sample cap: PATH_POOL_SAMPLES / n_channels. */
int config_path_per_axis_max(void);

/* True if gpio_level (0/1) matches the pin's active setting. */
bool config_pin_asserted(int gpio_level, int active);

#ifdef __cplusplus
}
#endif
