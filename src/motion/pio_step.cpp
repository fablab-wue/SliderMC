#include "pio_step.h"
#include "config_store.h"
#include "pins.h"
#include "debug_hw.h"
#include "motion_diag.h"
#include "axis_hw.h"

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
#define AXIS_MAX 2

typedef struct {
  int sm;
  uint offset;
  bool have_prog;
  bool active_high;
  bool running;
  PioStepWord shadow[SHADOW_MAX];
  unsigned shadow_n;
  int pending_steps;
  uint16_t instr[PROG_LEN];
  pio_program_t prog;
} PioAxis;

static PIO g_pio = pio0;
static PioAxis g_ax[AXIS_MAX];
static TaskHandle_t g_feed_task;
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

static bool axis_ok(int axis) { return axis >= 0 && axis < AXIS_MAX && g_ax[axis].sm >= 0; }

static void build_instr(PioAxis *a, bool active_high, uint base) {
  uint hi = active_high ? 1u : 0u;
  uint lo = active_high ? 0u : 1u;
  uint pulse = base + 3u;
  uint delay_loop = base + 12u;

  a->instr[0] = (uint16_t)pio_encode_out(pio_x, 26);
  a->instr[1] = (uint16_t)pio_encode_mov(pio_isr, pio_x);
  a->instr[2] = (uint16_t)pio_encode_out(pio_y, 6);
  a->instr[3] = (uint16_t)(pio_encode_set(pio_pins, hi) | pio_encode_delay(31));
  a->instr[4] = (uint16_t)(pio_encode_nop() | pio_encode_delay(31));
  a->instr[5] = (uint16_t)(pio_encode_nop() | pio_encode_delay(31));
  a->instr[6] = (uint16_t)(pio_encode_nop() | pio_encode_delay(31));
  a->instr[7] = (uint16_t)(pio_encode_nop() | pio_encode_delay(31));
  a->instr[8] = (uint16_t)(pio_encode_nop() | pio_encode_delay(31));
  a->instr[9] = (uint16_t)(pio_encode_nop() | pio_encode_delay(7));
  a->instr[10] = (uint16_t)pio_encode_set(pio_pins, lo);
  a->instr[11] = (uint16_t)pio_encode_mov(pio_x, pio_isr);
  a->instr[12] = (uint16_t)pio_encode_jmp_x_dec(delay_loop);
  a->instr[13] = (uint16_t)pio_encode_jmp_y_dec(pulse);
}

static void notify_feed_from_isr(void) {
  if (!g_feed_task) {
    return;
  }
  BaseType_t hpw = pdFALSE;
  vTaskNotifyGiveFromISR(g_feed_task, &hpw);
  portYIELD_FROM_ISR(hpw);
}

static enum pio_interrupt_source tx_irq_source(int axis) {
  return (enum pio_interrupt_source)(pis_sm0_tx_fifo_not_full + (uint)g_ax[axis].sm);
}

static void __isr pio_irq_handler(void) {
  for (int a = 0; a < AXIS_MAX; ++a) {
    if (g_ax[a].sm >= 0) {
      pio_set_irq0_source_enabled(g_pio, tx_irq_source(a), false);
    }
  }
  dbg_hw_set(PIN_DBG_IRQ, 1);
  dbg_hw_set(PIN_DBG_IRQ, 0);
  notify_feed_from_isr();
}

void pio_step_arm_tx_irq(void) {
  for (int a = 0; a < AXIS_MAX; ++a) {
    if (g_ax[a].sm >= 0) {
      pio_set_irq0_source_enabled(g_pio, tx_irq_source(a), true);
    }
  }
}

void pio_step_disarm_tx_irq(void) {
  for (int a = 0; a < AXIS_MAX; ++a) {
    if (g_ax[a].sm >= 0) {
      pio_set_irq0_source_enabled(g_pio, tx_irq_source(a), false);
    }
  }
}

void pio_step_kick_feed(void) {
  if (g_feed_task) {
    xTaskNotifyGive(g_feed_task);
  }
}

static void unload_axis(int axis) {
  PioAxis *a = &g_ax[axis];
  if (a->sm >= 0) {
    pio_set_irq0_source_enabled(g_pio, tx_irq_source(axis), false);
    pio_sm_set_enabled(g_pio, (uint)a->sm, false);
    pio_sm_unclaim(g_pio, (uint)a->sm);
    a->sm = -1;
  }
  if (a->have_prog) {
    pio_remove_program(g_pio, &a->prog, a->offset);
    a->have_prog = false;
  }
}

static bool load_axis_sm(int axis, bool active_high) {
  PioAxis *a = &g_ax[axis];
  unload_axis(axis);
  a->active_high = active_high;

  a->sm = (int)pio_claim_unused_sm(g_pio, false);
  if (a->sm < 0) {
    return false;
  }

  build_instr(a, active_high, 0);
  a->prog.instructions = a->instr;
  a->prog.length = PROG_LEN;
  a->prog.origin = -1;
  if (!pio_can_add_program(g_pio, &a->prog)) {
    pio_sm_unclaim(g_pio, (uint)a->sm);
    a->sm = -1;
    return false;
  }
  a->offset = pio_add_program(g_pio, &a->prog);
  a->have_prog = true;

  build_instr(a, active_high, a->offset);
  for (uint i = 0; i < PROG_LEN; ++i) {
    g_pio->instr_mem[a->offset + i] = a->instr[i];
  }

  const int step_pin = axis_hw_step_pin(axis);
  const int dir_pin = axis_hw_dir_pin(axis);

  pio_gpio_init(g_pio, step_pin);
  gpio_set_drive_strength(step_pin, GPIO_DRIVE_STRENGTH_8MA);
  pio_sm_set_consecutive_pindirs(g_pio, (uint)a->sm, step_pin, 1, true);

  pio_sm_config c = pio_get_default_sm_config();
  sm_config_set_set_pins(&c, step_pin, 1);
  sm_config_set_out_shift(&c, true, true, 32);
  sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
  sm_config_set_wrap(&c, a->offset + 0u, a->offset + (PROG_LEN - 1u));
  sm_config_set_clkdiv(&c, 2.5f);

  pio_sm_init(g_pio, (uint)a->sm, a->offset, &c);
  pio_sm_clear_fifos(g_pio, (uint)a->sm);
  pio_sm_set_enabled(g_pio, (uint)a->sm, false);

  pio_set_irq0_source_enabled(g_pio, tx_irq_source(axis), false);
  if (!g_irq_installed) {
    irq_set_exclusive_handler(PIO0_IRQ_0, pio_irq_handler);
    irq_set_enabled(PIO0_IRQ_0, true);
    g_irq_installed = true;
  }

  gpio_init(dir_pin);
  gpio_set_dir(dir_pin, GPIO_OUT);
  gpio_set_drive_strength(dir_pin, GPIO_DRIVE_STRENGTH_8MA);
  return true;
}

void pio_step_set_feed_task(void *task_handle) {
  g_feed_task = (TaskHandle_t)task_handle;
}

void pio_step_init(void) {
  for (int i = 0; i < AXIS_MAX; ++i) {
    g_ax[i].sm = -1;
    g_ax[i].have_prog = false;
    g_ax[i].shadow_n = 0;
    g_ax[i].pending_steps = 0;
    g_ax[i].running = false;
  }
  g_feed_task = nullptr;
  if (!g_pio_mu) {
    g_pio_mu = xSemaphoreCreateRecursiveMutex();
  }
  if (!load_axis_sm(0, axis_hw_step_active(0) != 0)) {
    return;
  }
  if (config_axis2_enabled()) {
    (void)load_axis_sm(1, axis_hw_step_active(1) != 0);
  }
}

bool pio_step_reconfigure(void) {
  for (int i = 0; i < AXIS_MAX; ++i) {
    if (g_ax[i].running || g_ax[i].shadow_n > 0 || g_ax[i].pending_steps > 0) {
      return false;
    }
  }
  if (!load_axis_sm(0, axis_hw_step_active(0) != 0)) {
    return false;
  }
  if (config_axis2_enabled()) {
    return load_axis_sm(1, axis_hw_step_active(1) != 0);
  }
  unload_axis(1);
  return true;
}

void pio_step_start(int axis) {
  if (!axis_ok(axis)) {
    return;
  }
  g_ax[axis].running = true;
  pio_sm_set_enabled(g_pio, (uint)g_ax[axis].sm, true);
}

void pio_step_stop_hard(int axis) {
  if (axis < 0 || axis >= AXIS_MAX) {
    return;
  }
  pio_lock();
  PioAxis *a = &g_ax[axis];
  if (a->sm >= 0) {
    pio_set_irq0_source_enabled(g_pio, tx_irq_source(axis), false);
    pio_sm_set_enabled(g_pio, (uint)a->sm, false);
    pio_sm_clear_fifos(g_pio, (uint)a->sm);
    pio_sm_restart(g_pio, (uint)a->sm);
    g_pio->fdebug = 1u << (PIO_FDEBUG_TXSTALL_LSB + (uint)a->sm);
  }
  a->running = false;
  a->shadow_n = 0;
  a->pending_steps = 0;
  if (axis == 0) {
    dbg_hw_set(PIN_DBG_FIFO, 0);
  }
  pio_unlock();
}

void pio_step_stop_soft(int axis) {
  if (axis < 0 || axis >= AXIS_MAX) {
    return;
  }
  pio_lock();
  PioAxis *a = &g_ax[axis];
  if (a->sm >= 0) {
    pio_set_irq0_source_enabled(g_pio, tx_irq_source(axis), false);
    if (pio_sm_is_tx_fifo_empty(g_pio, (uint)a->sm)) {
      pio_sm_set_enabled(g_pio, (uint)a->sm, false);
    }
    g_pio->fdebug = 1u << (PIO_FDEBUG_TXSTALL_LSB + (uint)a->sm);
  }
  a->running = false;
  a->shadow_n = 0;
  a->pending_steps = 0;
  if (axis == 0) {
    dbg_hw_set(PIN_DBG_FIFO, 0);
  }
  pio_unlock();
}

static void sync_shadow_to_fifo(int axis) {
  PioAxis *a = &g_ax[axis];
  if (a->sm < 0) {
    return;
  }
  unsigned lvl = pio_sm_get_tx_fifo_level(g_pio, (uint)a->sm);
  if (lvl == 0 && axis == 0) {
    dbg_hw_set(PIN_DBG_FIFO, 0);
  }
  while (a->shadow_n > lvl) {
    a->pending_steps -= (int)a->shadow[0].steps;
    if (a->pending_steps < 0) {
      a->pending_steps = 0;
    }
    memmove(&a->shadow[0], &a->shadow[1], (a->shadow_n - 1) * sizeof(a->shadow[0]));
    --a->shadow_n;
  }
}

bool pio_step_put_word(int axis, uint32_t delay_cycles, uint8_t n_pulses) {
  pio_lock();
  if (!axis_ok(axis) || n_pulses < 1) {
    pio_unlock();
    return n_pulses < 1;
  }
  PioAxis *a = &g_ax[axis];
  if (n_pulses > (PIO_STEP_REPEAT_MAX + 1)) {
    n_pulses = (uint8_t)(PIO_STEP_REPEAT_MAX + 1);
  }
  sync_shadow_to_fifo(axis);
  if (pio_sm_is_tx_fifo_full(g_pio, (uint)a->sm)) {
    pio_unlock();
    return false;
  }
  uint32_t delay = delay_cycles & PIO_STEP_DELAY_MASK;
  uint32_t repeat = (uint32_t)(n_pulses - 1) & 0x3Fu;
  uint32_t word = delay | (repeat << PIO_STEP_REPEAT_SHIFT);
  pio_sm_put(g_pio, (uint)a->sm, word);
  if (axis == 0) {
    dbg_hw_set(PIN_DBG_FIFO, 1);
    static int32_t last_delay = -1;
    dbg_hw_set(PIN_DBG_MOV_CONST, delay == (uint32_t)last_delay);
    last_delay = (int32_t)delay;
  }

  if (a->shadow_n < SHADOW_MAX) {
    a->shadow[a->shadow_n].steps = n_pulses;
    a->shadow[a->shadow_n].delay = delay;
    ++a->shadow_n;
    a->pending_steps += n_pulses;
  } else {
    pio_unlock();
    return false;
  }
  if (axis == 0) {
    motion_diag_note_fifo_level(pio_sm_get_tx_fifo_level(g_pio, (uint)a->sm));
  }
  pio_unlock();
  return true;
}

unsigned pio_step_tx_level(int axis) {
  if (!axis_ok(axis)) {
    return 0;
  }
  return pio_sm_get_tx_fifo_level(g_pio, (uint)g_ax[axis].sm);
}

unsigned pio_step_tx_room(int axis) {
  if (!axis_ok(axis)) {
    return 0;
  }
  return 8u - pio_sm_get_tx_fifo_level(g_pio, (uint)g_ax[axis].sm);
}

bool pio_step_tx_empty(int axis) {
  if (!axis_ok(axis)) {
    return true;
  }
  return pio_sm_is_tx_fifo_empty(g_pio, (uint)g_ax[axis].sm);
}

bool pio_step_is_stalled(int axis) {
  if (!axis_ok(axis) || !g_ax[axis].running) {
    return false;
  }
  uint32_t mask = 1u << (PIO_FDEBUG_TXSTALL_LSB + (uint)g_ax[axis].sm);
  if (g_pio->fdebug & mask) {
    g_pio->fdebug = mask;
    return true;
  }
  return false;
}

void pio_step_clear_stall(int axis) {
  if (!axis_ok(axis)) {
    return;
  }
  g_pio->fdebug = 1u << (PIO_FDEBUG_TXSTALL_LSB + (uint)g_ax[axis].sm);
}

int pio_step_pending_steps(int axis) {
  if (axis < 0 || axis >= AXIS_MAX) {
    return 0;
  }
  pio_lock();
  if (g_ax[axis].sm >= 0) {
    sync_shadow_to_fifo(axis);
  }
  int pending = g_ax[axis].pending_steps;
  pio_unlock();
  return pending;
}

void pio_step_set_dir(int axis, int sign_pos) {
  if (axis < 0 || axis >= AXIS_MAX) {
    return;
  }
  int active = axis_hw_dir_active(axis) ? 1 : 0;
  int level = (sign_pos >= 0) ? active : (active ? 0 : 1);
  gpio_put(axis_hw_dir_pin(axis), level);
}

uint32_t pio_step_sysclk_hz(void) {
  return 50000000u;
}

#else /* HOST_TEST */

static unsigned g_level[2];
static int g_pending[2];
static bool g_axis1;

void pio_step_init(void) {
  g_level[0] = g_level[1] = 0;
  g_pending[0] = g_pending[1] = 0;
  g_axis1 = config_axis2_enabled();
}
bool pio_step_reconfigure(void) {
  g_axis1 = config_axis2_enabled();
  return true;
}
void pio_step_start(int axis) { (void)axis; }
void pio_step_stop_hard(int axis) {
  if (axis >= 0 && axis < 2) {
    g_level[axis] = 0;
    g_pending[axis] = 0;
  }
}
void pio_step_stop_soft(int axis) {
  if (axis >= 0 && axis < 2) {
    g_level[axis] = 0;
    g_pending[axis] = 0;
  }
}
bool pio_step_put_word(int axis, uint32_t delay_cycles, uint8_t n_pulses) {
  (void)delay_cycles;
  if (axis < 0 || axis > 1 || (axis == 1 && !g_axis1)) {
    return n_pulses < 1;
  }
  if (n_pulses < 1) {
    return true;
  }
  if (g_level[axis] >= 8) {
    return false;
  }
  ++g_level[axis];
  g_pending[axis] += n_pulses;
  return true;
}
unsigned pio_step_tx_level(int axis) {
  return (axis >= 0 && axis < 2) ? g_level[axis] : 0;
}
unsigned pio_step_tx_room(int axis) {
  return (axis >= 0 && axis < 2) ? (8u - g_level[axis]) : 0;
}
bool pio_step_tx_empty(int axis) {
  return (axis < 0 || axis >= 2) || g_level[axis] == 0;
}
bool pio_step_is_stalled(int axis) {
  (void)axis;
  return false;
}
void pio_step_clear_stall(int axis) { (void)axis; }
int pio_step_pending_steps(int axis) {
  return (axis >= 0 && axis < 2) ? g_pending[axis] : 0;
}
void pio_step_set_dir(int axis, int sign_pos) {
  (void)axis;
  (void)sign_pos;
}
uint32_t pio_step_sysclk_hz(void) { return 50000000u; }
void pio_step_set_feed_task(void *task_handle) { (void)task_handle; }
void pio_step_arm_tx_irq(void) {}
void pio_step_disarm_tx_irq(void) {}
void pio_step_kick_feed(void) {}

#endif
