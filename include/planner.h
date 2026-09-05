#pragma once

#include "motion_api.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void planner_init(void);

/**
 * Highest-prio path (feed task only): issue STEP words from stop-distance law.
 * Returns the number of words queued, so the caller can tell "FIFO full" from
 * "planner has nothing to add" and sleep instead of spinning.
 */
int planner_fill_fifo(int axis);

/** ~200 Hz: switches, DIR pause, settle, underrun check, status fields (no FIFO fill). */
void planner_tick(float dt_s);

void planner_request_move_to(int axis, float mm);
void planner_request_move_by(int axis, float mm);
void planner_request_jog(int axis, int dir);
void planner_request_stop(void);  /* soft-stop all axes */
void planner_request_halt(void);  /* hard halt all axes */
void planner_request_home(int axis);
void planner_soft_reset(void);

/** Override cruise/accel for one axis (coordinated MT). */
void planner_set_cruise_accel(int axis, float cruise_mm_s, float accel_mm_s2);

/**
 * Seed position/velocity from motion_path (2nd planner) before a following
 * planner_request_stop()/planner_request_halt() decelerates from that speed.
 */
void planner_takeover_from_path(int axis, int64_t pos_steps, float vel_mm_s);

/** Redefine the step counter so IP reports mm (no motion). */
void planner_set_position(int axis, float mm);

/** True if signed move (neg=left, pos=right) is blocked by hard limit. */
bool planner_hard_limit_blocks_sign(int axis, int sign);

bool planner_is_busy(void);
bool planner_is_moving(void);
/** True while any active axis wants FIFO words (moving, no DIR pause). */
bool planner_feed_active(void);
/** True while this axis wants FIFO words. */
bool planner_feed_active_axis(int axis);
void planner_get_status(McStatus *out);

#ifdef __cplusplus
}
#endif
