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

static void task_motion_feed(void *arg) {
  (void)arg;
  for (;;) {
    if (motion_path_is_active()) {
      /* Path-mode (2nd planner): fill until TX is full or path has nothing more. */
      for (int i = 0; i < 32; ++i) {
        if (pio_step_tx_room() == 0 || motion_path_fill_fifo() == 0) {
          break;
        }
        if (!motion_path_is_active()) {
          break;
        }
      }
      if (pio_step_tx_room() == 0) {
        pio_step_arm_tx_irq();
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(2));
      } else {
        pio_step_disarm_tx_irq();
        vTaskDelay(1);
      }
    } else if (planner_feed_active()) {
      /* Fill until TX is full or the planner has nothing more to queue. */
      for (int i = 0; i < 32; ++i) {
        if (pio_step_tx_room() == 0 || planner_fill_fifo() == 0) {
          break;
        }
        if (!planner_feed_active()) {
          break;
        }
      }
      /*
       * TXNFULL is level-triggered: arming it while the FIFO has room re-fires
       * immediately. Only wait on it when the FIFO is actually full, otherwise
       * sleep — this task outranks `plan`/`proto`, so it must never spin.
       */
      if (pio_step_tx_room() == 0) {
        pio_step_arm_tx_irq();
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(2));
      } else {
        pio_step_disarm_tx_irq();
        vTaskDelay(1);
      }
    } else if (planner_is_moving()) {
      /* DIR-change pause: an armed TX IRQ on an empty FIFO would spin here. */
      pio_step_disarm_tx_irq();
      vTaskDelay(pdMS_TO_TICKS(2));
    } else {
      /* Block until a move kicks the feed task (or rare timeout). */
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
  board_heartbeat_init(); /* LED + optional WDT before unlock wait */
  board_wait_unlock_newline(); /* unlock: LF from UIC UART or USB CDC */
  protocol_send_banner();
  board_heartbeat_ready(); /* McState LED patterns from here */
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
  xTaskCreate(task_motion_feed, "feed", 1536, nullptr, configMAX_PRIORITIES - 1,
              &g_feed_handle);
  pio_step_set_feed_task(g_feed_handle);
  xTaskCreate(task_planner, "plan", 2048, nullptr, configMAX_PRIORITIES - 2,
              nullptr);
  xTaskCreate(task_protocol, "proto", 4096, nullptr, tskIDLE_PRIORITY + 2,
              nullptr);
}

#endif /* !HOST_TEST */
