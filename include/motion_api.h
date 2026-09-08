#pragma once

#include <stdbool.h>

#include "config_defaults.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  MC_STATE_DISABLED = 0,
  MC_STATE_IDLE,
  MC_STATE_ACCELERATING,
  MC_STATE_MOVING,
  MC_STATE_DECELERATING,
  MC_STATE_HOMING,
  MC_STATE_HARD_LIMIT,
  MC_STATE_ERROR,
  MC_STATE_PATH,
  MC_STATE_LOCKED,   // must be last for heart beat logic
} McState;

typedef struct {
  McState state;
  bool enabled;
  bool moving;
  bool homing;
  bool at_soft_limit;
  bool hard_limit; /* latched or switch still asserted (debounced) */
  bool drv_error;
  bool has_target;
  /* Packed motors then servos (length config_axis_count()). */
  float pos[MC_CH_MAX];
  float target[MC_CH_MAX];
  float vel[MC_CH_MAX];
  float acc[MC_CH_MAX];
} McStatus;

void motion_init(void);
/** Re-pack servo boot poses after CS motors/servos. */
void motion_on_counts_changed(void);

bool motion_enable(bool on);
bool motion_move_to(float mm); /* packed channel 0 only */
/** Absolute move; NAN = skip that packed channel. Followers scale to the master duration. */
bool motion_move_to2(float mm1_or_nan, float mm2_or_nan);
bool motion_move_to_n(const float dest[MC_CH_MAX]);
bool motion_move_by(float mm); /* packed channel 0 only */
/**
 * Signed joy percent of session SS per packed channel. NAN = 0 on that channel.
 * Does not change session SS.
 */
bool motion_joy(const float pct[MC_CH_MAX]);
/** Leave joy-mode without issuing a stop (another motion source takes over). */
void motion_end_joy(void);
bool motion_stop(void);   /* soft stop all */
bool motion_halt(void);   /* error-decel halt all */
/** Homing cycle for motor 1, 2 or 3 (1-based). Servos are not homeable. */
bool motion_home(int axis_1based);
bool motion_soft_reset(void); /* clear alarm / reset hold */
/** Redefine reported pose. NAN = leave that packed channel. Idle only. */
bool motion_set_position(const float mm[MC_CH_MAX]);

bool motion_set_speed(float mm_s);
bool motion_set_accel(float mm_s2);
bool motion_set_max_speed(float mm_s);
/** Session working window (SL/SR). set[i]=false skips; set+NAN stores None. false = !E:limit. */
bool motion_set_window_left(const bool set[MC_CH_MAX], const float mm[MC_CH_MAX]);
bool motion_set_window_right(const bool set[MC_CH_MAX], const float mm[MC_CH_MAX]);
void motion_reset_window_left(void);
void motion_reset_window_right(void);

void motion_get_status(McStatus *out);
bool motion_is_busy(void); /* moving or homing on any channel */
/** Packed channel used as time-sync / WP master, or -1 if none. */
int motion_master_channel(void);

/* Host/test helper: advance stub time so waits can complete. */
void motion_stub_tick_ms(unsigned ms);

#ifdef HOST_TEST
/** Per-axis cruise/accel after dual-MT ratio (for host tests). */
float motion_host_axis_cruise(int axis);
float motion_host_axis_accel(int axis);
#endif

#ifdef __cplusplus
}
#endif
