#include "motion_task.h"
#include "planner.h"
#include "pio_step.h"
#include "board.h"
#include "board_heartbeat.h"
#include "protocol.h"
#include "motion_diag.h"
#include "config_store.h"
#include "motion_path.h"

#ifndef HOST_TEST

#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>

static TaskHandle_t g_feed_handle;

static bool any_tx_full(void) {
  int n = config_motor_count();
  for (int a = 0; a < n; ++a) {
    if (pio_step_tx_room(a) == 0) {
      return true;
    }
  }
  return false;
}

static bool any_fifo_low(void) {
  int n = config_motor_count();
  for (int a = 0; a < n; ++a) {
    if (!planner_feed_active_axis(a)) {
      continue;
    }
    unsigned lvl = pio_step_tx_level(a);
    /* Keep a margin above the SM prefill threshold and react before the FIFO
     * reaches the stall edge. The 3-axis case drains faster than a single 1 ms
     * sleep budget can recover, so refill must happen as soon as a lane drops
     * back toward the startup buffer. */
    if (lvl <= PIO_STEP_START_MIN_LEVEL + 2u) {
      return true;
    }
  }
  return false;
}

/*
 * Fill policy. 1 = lowest TX FIFO first (trial). 0 = previous round-robin
 * (the block in the #else below is the pre-trial source; flip this to revert).
 */
#ifndef PLANNER_FEED_LOWEST_FIFO
#define PLANNER_FEED_LOWEST_FIFO 1
#endif

#if PLANNER_FEED_LOWEST_FIFO
/* Among axes that want words and have TX room, pick the driest FIFO.
 * Equal levels keep a rotating tie-break so axis 0 is not always first.
 * `skip` is per-axis for this wake (time-budget / nothing to emit). */
static int feed_pick_lowest_fifo(int n, int tie_from, const bool *skip) {
  int best = -1;
  unsigned best_lvl = 0xffffffffu;
  int best_rr = 0;
  for (int a = 0; a < n; ++a) {
    if (skip[a] || !planner_feed_active_axis(a) || pio_step_tx_room(a) == 0) {
      continue;
    }
    unsigned lvl = pio_step_tx_level(a);
    int rr = (a - tie_from + n) % n;
    if (best < 0 || lvl < best_lvl || (lvl == best_lvl && rr < best_rr)) {
      best = a;
      best_lvl = lvl;
      best_rr = rr;
    }
  }
  return best;
}
#endif

static void task_motion_feed(void *arg) {
  (void)arg;
  for (;;) {
    if (motion_path_is_active()) {
      for (int i = 0; i < 32; ++i) {
        if (motion_path_fill_fifo() == 0) {
          break;
        }
        if (!motion_path_is_active()) {
          break;
        }
      }
      if (any_tx_full()) {
        pio_step_arm_tx_irq();
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1));
      } else {
        pio_step_disarm_tx_irq();
        vTaskDelay(pdMS_TO_TICKS(1));
      }
    } else if (planner_feed_active()) {
      static int rr = 0;
      int n = config_motor_count();
#if PLANNER_FEED_LOWEST_FIFO
      /* Re-pick the driest FIFO after every fill. 32*n matches the old
       * 32-pass × n-axis word budget. */
      int passes = (n > 0) ? (32 * n) : 0;
      bool skip[3] = {false, false, false};
      for (int i = 0; i < passes; ++i) {
        int a = feed_pick_lowest_fifo(n, rr, skip);
        if (a < 0) {
          break;
        }
        int emitted = planner_fill_fifo(a);
        rr = (a + 1) % n;
        if (emitted == 0) {
          skip[a] = true;
          continue;
        }
        if (!planner_feed_active()) {
          break;
        }
      }
#else
      /* Previous round-robin (pre lowest-FIFO trial). */
      for (int i = 0; i < 32; ++i) {
        bool any_want = false;
        int emitted = 0;
        for (int k = 0; k < n; ++k) {
          int a = (rr + k) % n;
          if (planner_feed_active_axis(a) && pio_step_tx_room(a) > 0) {
            any_want = true;
            emitted += planner_fill_fifo(a);
          }
        }
        if (n > 0) {
          rr = (rr + 1) % n;
        }
        if (!any_want) {
          break;
        }
        if (emitted == 0) {
          break;
        }
        if (!planner_feed_active()) {
          break;
        }
      }
#endif
      if (any_tx_full()) {
        pio_step_arm_tx_irq();
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1));
      } else if (any_fifo_low()) {
        /* Keep the IRQ armed while we are in the low-water region and immediately
         * re-check the queue after each wake so the refill loop is driven by the
         * actual FIFO occupancy, not by the nominal 1 ms scheduling tick. */
        pio_step_arm_tx_irq();
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1));
        continue;
      } else if (planner_feed_active()) {
        /* 3-axis startup and handoff is the tightest edge: keep the refill loop
         * awake during the first packets of each move so the FIFO never waits for
         * the later low-water wake before the next burst is scheduled. */
        pio_step_arm_tx_irq();
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1));
        continue;
      } else {
        pio_step_disarm_tx_irq();
        vTaskDelay(pdMS_TO_TICKS(1));
      }
    } else if (planner_is_moving()) {
      pio_step_disarm_tx_irq();
      vTaskDelay(pdMS_TO_TICKS(2));
    } else {
      pio_step_disarm_tx_irq();
      ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(50));
    }
  }
}

static void task_planner(void *arg) {
  (void)arg;
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    TickType_t now = xTaskGetTickCount();
    float dt = (now - last) * portTICK_PERIOD_MS * 1e-3f;
    last = now;
    if (dt <= 0.0f) {
      dt = 0.001f;
    } else if (dt > 0.05f) {
      dt = 0.05f;
    }
    planner_tick(dt);
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

static void proto_write_both(const char *data, size_t n, void *ctx) {
  (void)ctx;
  board_usb_write(data, n);
  board_uart_write(data, n);
}

static void proto_write_debug(const char *data, size_t n, void *ctx) {
  (void)ctx;
  board_usb_debug(data, n);
}

static void task_protocol(void *arg) {
  (void)arg;
  ProtocolIo io = {proto_write_both, proto_write_debug, nullptr};
  protocol_init(io);
  board_heartbeat_init();
  board_wait_unlock_newline();
  protocol_send_banner();
  board_heartbeat_ready();
  motion_diag_boot_report();

  TickType_t last = xTaskGetTickCount();
  for (;;) {
    while (Serial.available() > 0) {
      protocol_feed_byte((uint8_t)Serial.read());
    }
    board_uart_poll_rx();

    TickType_t now = xTaskGetTickCount();
    unsigned dt = (unsigned)((now - last) * portTICK_PERIOD_MS);
    last = now;
    if (dt == 0) {
      dt = 1;
    }
    protocol_poll(dt);
    board_heartbeat_tick(dt);
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void motion_tasks_start(void) {
  xTaskCreate(task_motion_feed, "feed", 2048, nullptr, configMAX_PRIORITIES - 1,
              &g_feed_handle);
  pio_step_set_feed_task(g_feed_handle);
  xTaskCreate(task_planner, "plan", 2048, nullptr, configMAX_PRIORITIES - 2,
              nullptr);
  xTaskCreate(task_protocol, "proto", 4096, nullptr, tskIDLE_PRIORITY + 2,
              nullptr);
}

#endif /* !HOST_TEST */
