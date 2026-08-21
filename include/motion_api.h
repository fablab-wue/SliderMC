#pragma once

#include <stdbool.h>

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
  MC_STATE_LOCKED,
  MC_STATE_PATH,
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
  float pos_mm;
  float pos_mm_2; /* axis2 position when axis2 enabled; else 0 */
  float target_mm;
  float target_mm_2;
  float vel_mm_s;
  float vel_mm_s_2;
  float acc_mm_s2;
  float acc_mm_s2_2;
} McStatus;

void motion_init(void);

bool motion_enable(bool on);
bool motion_move_to(float mm); /* axis 0 only */
/** Absolute move; NAN = skip that axis. When both move, axis1 speed/accel scaled to match duration. */
bool motion_move_to2(float mm1_or_nan, float mm2_or_nan);
bool motion_move_by(float mm); /* axis 0 only */
/**
 * Continuous jog. axis_mask: 0=both (or axis0 if 1-axis mode), 1=axis0, 2=axis1.
 * In 1-axis mode the mask is ignored.
 */
bool motion_jog(int dir, int axis_mask);
bool motion_stop(void);   /* soft stop both */
bool motion_halt(void);   /* error-decel halt both */
/** Homing cycle for axis 1 or 2 (1-based). Default callers use 1. */
bool motion_home(int axis_1based);
bool motion_soft_reset(void); /* clear alarm / reset hold */

bool motion_set_speed(float mm_s);
bool motion_set_accel(float mm_s2);
bool motion_set_max_speed(float mm_s);
bool motion_set_soft_limits(bool min_en, float min_mm, bool max_en, float max_mm);

void motion_get_status(McStatus *out);
bool motion_is_busy(void); /* moving or homing on either axis */

/* Host/test helper: advance stub time so WM/WH can complete. */
void motion_stub_tick_ms(unsigned ms);

#ifdef HOST_TEST
/** Per-axis cruise/accel after dual-MT ratio (for host tests). */
float motion_host_axis_cruise(int axis);
float motion_host_axis_accel(int axis);
#endif

#ifdef __cplusplus
}
#endif
