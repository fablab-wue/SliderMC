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
int planner_fill_fifo(void);

/** ~200 Hz: switches, DIR pause, settle, underrun check, status fields (no FIFO fill). */
void planner_tick(float dt_s);

void planner_request_move_to(float mm);
void planner_request_move_by(float mm);
void planner_request_jog(int dir);
void planner_request_stop(void);
void planner_request_halt(void);
void planner_request_home(void);
void planner_soft_reset(void);

/**
 * Seed position/velocity from motion_path (2nd planner) before a following
 * planner_request_stop()/planner_request_halt() decelerates from that speed.
 */
void planner_takeover_from_path(int64_t pos_steps, float vel_mm_s);

/** True if signed move (neg=left, pos=right) is blocked by hard limit. */
bool planner_hard_limit_blocks_sign(int sign);

/** Debounced PIN_SW_HOME asserted (SW_HOME_use=1; for MH only, never a halt). */
bool planner_home_switch_asserted(void);

/** True when SW_HOME_use is enabled in config. */
bool planner_home_switch_enabled(void);

bool planner_is_busy(void);
bool planner_is_moving(void);
/** True while the planner actually wants FIFO words (moving, no DIR pause). */
bool planner_feed_active(void);
void planner_get_status(McStatus *out);

#ifdef __cplusplus
}
#endif
