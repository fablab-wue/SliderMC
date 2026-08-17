#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/** Start FreeRTOS tasks: MotionFeed (highest), Planner, Protocol, Idle. */
void motion_tasks_start(void);

#ifdef __cplusplus
}
#endif
