#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Start onboard LED heartbeat and arm WDT if WDT_use=1.
 * Call before unlock wait so the wait-for-\n LED pattern can run.
 */
void board_heartbeat_init(void);

/**
 * Mark session unlocked (after banner). LED patterns then follow McState.
 */
void board_heartbeat_ready(void);

/**
 * Feed WDT every call; advance LED/counter every 3rd call (~67 Hz on a 5 ms poll).
 * dt_ms is unused for LED cadence (callers may still pass elapsed ms).
 */
void board_heartbeat_tick(unsigned dt_ms);

#ifdef __cplusplus
}
#endif
