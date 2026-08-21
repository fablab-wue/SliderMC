#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * PIO STEP generator (per-axis SM).
 *
 * Word = delay[25:0] | (repeat[5:0] << 26); repeat R → R+1 pulses.
 * High phase ~1.5 µs @ 125 MHz (allows ≥300 kHz with short delay).
 * Polarity from config DRV_STEP_active / _2 via pio_step_reconfigure().
 *
 * axis: 0 = primary, 1 = optional 2nd axis (when config_axis2_enabled).
 */

#define PIO_STEP_REPEAT_SHIFT 26
#define PIO_STEP_DELAY_MASK 0x03FFFFFFu
#define PIO_STEP_REPEAT_MAX 63
#define PIO_STEP_HIGH_CYCLES 188u
#define PIO_STEP_SM_CLKDIV 2.5f /* 125 MHz / 2.5 = 50 MHz effective PIO SM clock */
/* Full period overhead before delay loop: high + SET lo + MOV X,ISR + JMP X--(+1) + JMP Y-- */
#define PIO_STEP_PERIOD_FIXED 192u

typedef struct {
  uint16_t steps;
  uint32_t delay;
} PioStepWord;

void pio_step_init(void);
bool pio_step_reconfigure(void);

void pio_step_start(int axis);
void pio_step_stop_hard(int axis);
void pio_step_stop_soft(int axis);

/** Non-blocking put; false if TX full. Advances shadow bookkeeping. */
bool pio_step_put_word(int axis, uint32_t delay_cycles, uint8_t n_pulses);

unsigned pio_step_tx_level(int axis);
unsigned pio_step_tx_room(int axis);
bool pio_step_tx_empty(int axis);
bool pio_step_is_stalled(int axis);

/** Drop a pending TXSTALL flag (e.g. after an intentional idle gap). */
void pio_step_clear_stall(int axis);

int pio_step_pending_steps(int axis);

/** Set DIR GPIO for positive (+mm) travel sense using DRV_DIR_active / _2. */
void pio_step_set_dir(int axis, int sign_pos);

uint32_t pio_step_sysclk_hz(void);

void pio_step_set_feed_task(void *task_handle); /* TaskHandle_t */

/**
 * Re-enable the TX-not-full interrupt for all active SMs. The handler masks
 * level sources, so the feed task must arm again right before it blocks.
 */
void pio_step_arm_tx_irq(void);

/** Mask TX-not-full IRQ on all active SMs. */
void pio_step_disarm_tx_irq(void);

/** Wake the feed task (e.g. when a move starts while it was idle-blocked). */
void pio_step_kick_feed(void);

#ifdef __cplusplus
}
#endif
