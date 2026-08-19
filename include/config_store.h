#pragma once

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
  float steps_per_mm;
  float slider_min_mm; /* NAN = disabled */
  float slider_max_mm; /* NAN = disabled */
  int init_verbose;
  int verbose_rate_hz; /* verbose `#…` push rate (1..200; proto task polls at 200 Hz) */
  int init_terminal; /* Terminal Mode (local echo) */
  int init_debug_level;
  int wdt_use; /* 1 = arm RP2040 WDT (2 s) from protocol heartbeat */

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
  int ext_active[10]; /* EXT_0..9_active: 0=low-active, 1=high-active */

  /* Homing: 0=off, 1=SW_HOME left, 2=SW_HOME right, 3=LIMIT_L, 4=LIMIT_R */
  int home_mode;
  float home_move_out_mm;
  float home_speed_mm_s;
  float home_accel_mm_s2;

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
} McSession;

void config_init_defaults(void);
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

/* Returns true on success. */
bool config_set_key(const char *key, const char *value);
bool config_get_key(const char *key, char *out, size_t out_len);

/* Emit all keys via callback: void cb(const char *key, const char *value, void *ctx) */
typedef void (*config_foreach_fn)(const char *key, const char *value, void *ctx);
void config_foreach(config_foreach_fn fn, void *ctx);

bool config_slider_min_enabled(void);
bool config_slider_max_enabled(void);

/* True if gpio_level (0/1) matches the pin's active setting. */
bool config_pin_asserted(int gpio_level, int active);

#ifdef __cplusplus
}
#endif
