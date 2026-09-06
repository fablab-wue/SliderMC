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
 * High phase ~3.8 µs @ 50 MHz SM clock (clk_sys / clkdiv). Max ≈ 259 kHz.
 * Polarity programs (active-high / active-low) are shared across SMs.
 * Axis 3 STEP polarity follows axis 2.
 *
 * axis: 0 = primary, 1/2 = optional extras when config axis >= 2/3.
 */

#define PIO_STEP_REPEAT_SHIFT 26
#define PIO_STEP_DELAY_MASK 0x03FFFFFFu
#define PIO_STEP_REPEAT_MAX 63
#define PIO_STEP_HIGH_CYCLES 188u
#define PIO_STEP_SM_HZ 50000000u /* STEP SM clock; clkdiv = clk_sys / this */
#ifndef PIO_STEP_SYSCLK_KHZ
#if defined(PICO_RP2350)
#define PIO_STEP_SYSCLK_KHZ 150000 /* RP2350 in-spec default */
#else
#define PIO_STEP_SYSCLK_KHZ 133000 /* RP2040 in-spec */
#endif
#endif
/* Full period overhead before delay loop: high + SET lo + MOV X,ISR + JMP X--(+1) + JMP Y-- */
#define PIO_STEP_PERIOD_FIXED 192u

typedef struct {
  uint16_t steps;
  uint32_t delay;
} PioStepWord;

void pio_step_init(void);
bool pio_step_reconfigure(void);
/** False if an enabled axis failed to claim an SM / program. */
bool pio_step_ok(void);

/* 3-axis starts are the tightest edge: start from a partial FIFO, not a full one.
 * Waiting for all 8 words before enabling the SM makes the very first handoff
 * stall while the feed task is still trying to refill. */
#define PIO_STEP_START_MIN_LEVEL 4u /* prefill this many words before enabling SM */
#if PIO_STEP_START_MIN_LEVEL >= 8u
#error "PIO_STEP_START_MIN_LEVEL must stay below the 8-word TX FIFO depth"
#endif

void pio_step_start(int axis);
/** Enable SM only after TX has PIO_STEP_START_MIN_LEVEL words (avoids launch stall). */
void pio_step_start_if_ready(int axis);
bool pio_step_is_running(int axis);
void pio_step_stop_hard(int axis);
void pio_step_stop_soft(int axis);

/** Non-blocking put; false if TX full. Advances shadow bookkeeping. */
bool pio_step_put_word(int axis, uint32_t delay_cycles, uint8_t n_pulses);

unsigned pio_step_tx_level(int axis);
unsigned pio_step_tx_room(int axis);
bool pio_step_tx_empty(int axis);
/**
 * True only when TXSTALL is set *and* issued work is still on the books
 * (shadow/pending) while the HW FIFO is already empty — a mid-burst dry-out.
 * TXSTALL + empty + pending==0 is the normal gap between fill bursts; that
 * must not count (and must not print). See pio_step_is_stalled() in pio_step.cpp.
 */
bool pio_step_is_stalled(int axis);

/** Drop a pending TXSTALL flag (e.g. after an intentional idle gap). */
void pio_step_clear_stall(int axis);

int pio_step_pending_steps(int axis);

/** Set DIR GPIO for positive (+mm) travel sense using DRV_DIR_N_active. */
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
