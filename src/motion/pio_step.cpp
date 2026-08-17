#include "pio_step.h"
#include "config_store.h"
#include "pins.h"
#include "debug_hw.h"
#include "motion_diag.h"

#ifndef HOST_TEST

#include <Arduino.h>
#include <string.h>

#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pio_instructions.h"
#include "hardware/irq.h"

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

#define SHADOW_MAX 8
#define PROG_LEN 14

static PIO g_pio = pio0;
static int g_sm = -1;
static uint g_offset = 0;
static bool g_have_prog = false;
static bool g_active_high = true;
static bool g_running = false;

static PioStepWord g_shadow[SHADOW_MAX];
static unsigned g_shadow_n;
static int g_pending_steps;
static TaskHandle_t g_feed_task;
static uint16_t g_instr[PROG_LEN];
static pio_program_t g_prog;
static bool g_irq_installed;
static SemaphoreHandle_t g_pio_mu;

static void pio_lock(void) {
  if (g_pio_mu) {
    xSemaphoreTakeRecursive(g_pio_mu, portMAX_DELAY);
  }
}

static void pio_unlock(void) {
  if (g_pio_mu) {
    xSemaphoreGiveRecursive(g_pio_mu);
  }
}

static void build_instr(bool active_high, uint base) {
  uint hi = active_high ? 1u : 0u;
  uint lo = active_high ? 0u : 1u;
  uint pulse = base + 3u;
  uint delay_loop = base + 12u;

  g_instr[0] = (uint16_t)pio_encode_out(pio_x, 26);
  g_instr[1] = (uint16_t)pio_encode_mov(pio_isr, pio_x);
  g_instr[2] = (uint16_t)pio_encode_out(pio_y, 6);
  /* High phase ~200 cycles @ 50 MHz: 32 + 5*32 + 8 */
  g_instr[3] = (uint16_t)(pio_encode_set(pio_pins, hi) | pio_encode_delay(31));
  g_instr[4] = (uint16_t)(pio_encode_nop() | pio_encode_delay(31));
  g_instr[5] = (uint16_t)(pio_encode_nop() | pio_encode_delay(31));
  g_instr[6] = (uint16_t)(pio_encode_nop() | pio_encode_delay(31));
  g_instr[7] = (uint16_t)(pio_encode_nop() | pio_encode_delay(31));
  g_instr[8] = (uint16_t)(pio_encode_nop() | pio_encode_delay(31));
  g_instr[9] = (uint16_t)(pio_encode_nop() | pio_encode_delay(7));
  g_instr[10] = (uint16_t)pio_encode_set(pio_pins, lo);
  g_instr[11] = (uint16_t)pio_encode_mov(pio_x, pio_isr);
  g_instr[12] = (uint16_t)pio_encode_jmp_x_dec(delay_loop);
  g_instr[13] = (uint16_t)pio_encode_jmp_y_dec(pulse);
}

static void notify_feed_from_isr(void) {
  if (!g_feed_task) {
    return;
  }
  BaseType_t hpw = pdFALSE;
  vTaskNotifyGiveFromISR(g_feed_task, &hpw);
  portYIELD_FROM_ISR(hpw);
}

static enum pio_interrupt_source tx_irq_source(void) {
  return (enum pio_interrupt_source)(pis_sm0_tx_fifo_not_full + (uint)g_sm);
}

/*
 * TXNFULL is a level source: it stays asserted while the FIFO has room, so it
 * must be masked in the handler and re-armed by the feed task before it blocks.
 * Leaving it enabled starves every other core activity (USB never enumerates).
 */
static void __isr pio_irq_handler(void) {
  if (g_sm >= 0) {
    pio_set_irq0_source_enabled(g_pio, tx_irq_source(), false);
  }
  dbg_hw_set(PIN_DBG_IRQ, 1);
  dbg_hw_set(PIN_DBG_IRQ, 0);
  notify_feed_from_isr();
}

void pio_step_arm_tx_irq(void) {
  if (g_sm < 0) {
    return;
  }
  pio_set_irq0_source_enabled(g_pio, tx_irq_source(), true);
}

void pio_step_disarm_tx_irq(void) {
  if (g_sm < 0) {
    return;
  }
  pio_set_irq0_source_enabled(g_pio, tx_irq_source(), false);
}

void pio_step_kick_feed(void) {
  if (g_feed_task) {
    xTaskNotifyGive(g_feed_task);
  }
}

static void unload_sm(void) {
  if (g_sm >= 0) {
    pio_set_irq0_source_enabled(g_pio, tx_irq_source(), false);
    pio_sm_set_enabled(g_pio, (uint)g_sm, false);
    pio_sm_unclaim(g_pio, (uint)g_sm);
    g_sm = -1;
  }
  if (g_have_prog) {
    pio_remove_program(g_pio, &g_prog, g_offset);
    g_have_prog = false;
  }
}

static bool load_pio_program(bool active_high) {
  unload_sm();
  g_active_high = active_high;

  g_sm = (int)pio_claim_unused_sm(g_pio, false);
  if (g_sm < 0) {
    return false;
  }

  /* Reserve space anywhere, then patch the jump targets for the real origin. */
  build_instr(active_high, 0);
  g_prog.instructions = g_instr;
  g_prog.length = PROG_LEN;
  g_prog.origin = -1;
  if (!pio_can_add_program(g_pio, &g_prog)) {
    pio_sm_unclaim(g_pio, (uint)g_sm);
    g_sm = -1;
    return false;
  }
  g_offset = pio_add_program(g_pio, &g_prog);
  g_have_prog = true;

  build_instr(active_high, g_offset);
  for (uint i = 0; i < PROG_LEN; ++i) {
    g_pio->instr_mem[g_offset + i] = g_instr[i];
  }

  pio_gpio_init(g_pio, PIN_DRV_STEP);
  gpio_set_drive_strength(PIN_DRV_STEP, GPIO_DRIVE_STRENGTH_8MA);
  pio_sm_set_consecutive_pindirs(g_pio, (uint)g_sm, PIN_DRV_STEP, 1, true);

  pio_sm_config c = pio_get_default_sm_config();
  sm_config_set_set_pins(&c, PIN_DRV_STEP, 1);
  sm_config_set_out_shift(&c, true, true, 32);
  sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
  sm_config_set_wrap(&c, g_offset + 0u, g_offset + (PROG_LEN - 1u));
  sm_config_set_clkdiv(&c, 2.5f);  // 125 MHz / 2.5 = 50 MHz

  pio_sm_init(g_pio, (uint)g_sm, g_offset, &c);
  pio_sm_clear_fifos(g_pio, (uint)g_sm);
  pio_sm_set_enabled(g_pio, (uint)g_sm, false);

  pio_set_irq0_source_enabled(g_pio, tx_irq_source(), false);
  if (!g_irq_installed) {
    irq_set_exclusive_handler(PIO0_IRQ_0, pio_irq_handler);
    irq_set_enabled(PIO0_IRQ_0, true);
    g_irq_installed = true;
  }

  gpio_init(PIN_DRV_DIR);
  gpio_set_dir(PIN_DRV_DIR, GPIO_OUT);
  gpio_set_drive_strength(PIN_DRV_DIR, GPIO_DRIVE_STRENGTH_8MA);
  return true;
}

void pio_step_set_feed_task(void *task_handle) {
  g_feed_task = (TaskHandle_t)task_handle;
}

void pio_step_init(void) {
  g_shadow_n = 0;
  g_pending_steps = 0;
  g_running = false;
  g_feed_task = nullptr;
  if (!g_pio_mu) {
    g_pio_mu = xSemaphoreCreateRecursiveMutex();
  }
  load_pio_program(config_get()->drv_step_active != 0);
}

bool pio_step_reconfigure(void) {
  if (g_running || g_shadow_n > 0 || g_pending_steps > 0) {
    return false;
  }
  return load_pio_program(config_get()->drv_step_active != 0);
}

void pio_step_start(void) {
  if (g_sm < 0) {
    return;
  }
  g_running = true;
  pio_sm_set_enabled(g_pio, (uint)g_sm, true);
}

void pio_step_stop_hard(void) {
  pio_lock();
  pio_step_disarm_tx_irq();
  if (g_sm >= 0) {
    pio_sm_set_enabled(g_pio, (uint)g_sm, false);
    pio_sm_clear_fifos(g_pio, (uint)g_sm);
    pio_sm_restart(g_pio, (uint)g_sm);
    pio_step_clear_stall();
  }
  g_running = false;
  g_shadow_n = 0;
  g_pending_steps = 0;
  dbg_hw_set(PIN_DBG_FIFO, 0);
  pio_unlock();
}

void pio_step_stop_soft(void) {
  pio_lock();
  pio_step_disarm_tx_irq();
  if (g_sm >= 0 && pio_sm_is_tx_fifo_empty(g_pio, (uint)g_sm)) {
    pio_sm_set_enabled(g_pio, (uint)g_sm, false);
  }
  pio_step_clear_stall();
  g_running = false;
  g_shadow_n = 0;
  g_pending_steps = 0;
  dbg_hw_set(PIN_DBG_FIFO, 0);
  pio_unlock();
}

static void sync_shadow_to_fifo(void) {
  if (g_sm < 0) {
    return;
  }
  unsigned lvl = pio_sm_get_tx_fifo_level(g_pio, (uint)g_sm);
  if (lvl == 0) {
    dbg_hw_set(PIN_DBG_FIFO, 0);
  }
  while (g_shadow_n > lvl) {
    g_pending_steps -= (int)g_shadow[0].steps;
    if (g_pending_steps < 0) {
      g_pending_steps = 0;
    }
    memmove(&g_shadow[0], &g_shadow[1], (g_shadow_n - 1) * sizeof(g_shadow[0]));
    --g_shadow_n;
  }
}

bool pio_step_put_word(uint32_t delay_cycles, uint8_t n_pulses) {
  pio_lock();
  if (g_sm < 0 || n_pulses < 1) {
    pio_unlock();
    return n_pulses < 1;
  }
  if (n_pulses > (PIO_STEP_REPEAT_MAX + 1)) {
    n_pulses = (uint8_t)(PIO_STEP_REPEAT_MAX + 1);
  }
  sync_shadow_to_fifo();
  if (pio_sm_is_tx_fifo_full(g_pio, (uint)g_sm)) {
    pio_unlock();
    return false;
  }
  uint32_t delay = delay_cycles & PIO_STEP_DELAY_MASK;
  uint32_t repeat = (uint32_t)(n_pulses - 1) & 0x3Fu;
  uint32_t word = delay | (repeat << PIO_STEP_REPEAT_SHIFT);
  pio_sm_put(g_pio, (uint)g_sm, word);
  dbg_hw_set(PIN_DBG_FIFO, 1);
  {
    static int32_t last_delay = -1;
    dbg_hw_set(PIN_DBG_MOV_CONST, delay == (uint32_t)last_delay);
    last_delay = (int32_t)delay;
  }

  if (g_shadow_n < SHADOW_MAX) {
    g_shadow[g_shadow_n].steps = n_pulses;
    g_shadow[g_shadow_n].delay = delay;
    ++g_shadow_n;
  }
  g_pending_steps += n_pulses;
  motion_diag_note_fifo_level(pio_sm_get_tx_fifo_level(g_pio, (uint)g_sm));
  pio_unlock();
  return true;
}

unsigned pio_step_tx_level(void) {
  if (g_sm < 0) {
    return 0;
  }
  return pio_sm_get_tx_fifo_level(g_pio, (uint)g_sm);
}

unsigned pio_step_tx_room(void) {
  if (g_sm < 0) {
    return 0;
  }
  return 8u - pio_sm_get_tx_fifo_level(g_pio, (uint)g_sm);
}

bool pio_step_tx_empty(void) {
  if (g_sm < 0) {
    return true;
  }
  return pio_sm_is_tx_fifo_empty(g_pio, (uint)g_sm);
}

bool pio_step_is_stalled(void) {
  if (g_sm < 0 || !g_running) {
    return false;
  }
  uint32_t mask = 1u << (PIO_FDEBUG_TXSTALL_LSB + (uint)g_sm);
  if (g_pio->fdebug & mask) {
    g_pio->fdebug = mask;
    return true;
  }
  return false;
}

void pio_step_clear_stall(void) {
  if (g_sm < 0) {
    return;
  }
  g_pio->fdebug = 1u << (PIO_FDEBUG_TXSTALL_LSB + (uint)g_sm);
}

int pio_step_pending_steps(void) {
  pio_lock();
  sync_shadow_to_fifo();
  int pending = g_pending_steps;
  pio_unlock();
  return pending;
}

void pio_step_set_dir(int sign_pos) {
  int active = config_get()->drv_dir_active ? 1 : 0;
  int level = (sign_pos >= 0) ? active : (active ? 0 : 1);
  gpio_put(PIN_DRV_DIR, level);
}

uint32_t pio_step_sysclk_hz(void) {
  return 50000000u;  // effective PIO SM clock
}

#else /* HOST_TEST */

static unsigned g_level;
static int g_pending;

void pio_step_init(void) {
  g_level = 0;
  g_pending = 0;
}
bool pio_step_reconfigure(void) { return true; }
void pio_step_start(void) {}
void pio_step_stop_hard(void) {
  g_level = 0;
  g_pending = 0;
}
void pio_step_stop_soft(void) {
  g_level = 0;
  g_pending = 0;
}
bool pio_step_put_word(uint32_t delay_cycles, uint8_t n_pulses) {
  (void)delay_cycles;
  if (n_pulses < 1) {
    return true;
  }
  if (g_level >= 8) {
    return false;
  }
  ++g_level;
  g_pending += n_pulses;
  return true;
}
unsigned pio_step_tx_level(void) { return g_level; }
unsigned pio_step_tx_room(void) { return 8u - g_level; }
bool pio_step_tx_empty(void) { return g_level == 0; }
bool pio_step_is_stalled(void) { return false; }
void pio_step_clear_stall(void) {}
int pio_step_pending_steps(void) { return g_pending; }
void pio_step_set_dir(int sign_pos) { (void)sign_pos; }
uint32_t pio_step_sysclk_hz(void) { return 50000000u; }
void pio_step_set_feed_task(void *task_handle) { (void)task_handle; }
void pio_step_arm_tx_irq(void) {}
void pio_step_disarm_tx_irq(void) {}
void pio_step_kick_feed(void) {}

#endif
