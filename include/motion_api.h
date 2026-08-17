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
  float target_mm;
  float vel_mm_s;
  float acc_mm_s2;
} McStatus;

void motion_init(void);

bool motion_enable(bool on);
bool motion_move_to(float mm);
bool motion_move_by(float mm);
bool motion_jog(int dir); /* -1 left, +1 right */
bool motion_stop(void);   /* soft stop */
bool motion_halt(void);   /* error-decel halt */
bool motion_home(void);
bool motion_soft_reset(void); /* clear alarm / reset hold */

bool motion_set_speed(float mm_s);
bool motion_set_accel(float mm_s2);
bool motion_set_max_speed(float mm_s);
bool motion_set_soft_limits(bool min_en, float min_mm, bool max_en, float max_mm);

void motion_get_status(McStatus *out);
bool motion_is_busy(void); /* moving or homing */

/* Host/test helper: advance stub time so WM/WH can complete. */
void motion_stub_tick_ms(unsigned ms);

#ifdef __cplusplus
}
#endif
