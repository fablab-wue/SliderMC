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
  if (pio_step_tx_room(0) == 0) {
    return true;
  }
  if (config_axis2_enabled() && pio_step_tx_room(1) == 0) {
    return true;
  }
  return false;
}

static void task_motion_feed(void *arg) {
  (void)arg;
  for (;;) {
    if (motion_path_is_active()) {
      for (int i = 0; i < 32; ++i) {
        if (any_tx_full() || motion_path_fill_fifo() == 0) {
          break;
        }
        if (!motion_path_is_active()) {
          break;
        }
      }
      if (any_tx_full()) {
        pio_step_arm_tx_irq();
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(2));
      } else {
        pio_step_disarm_tx_irq();
        vTaskDelay(1);
      }
    } else if (planner_feed_active()) {
      /* Fill each axis that has TX room independently. Do not stop the pass
       * just because one FIFO is full — the other may still be draining. */
      for (int i = 0; i < 32; ++i) {
        const bool want0 =
            planner_feed_active_axis(0) && pio_step_tx_room(0) > 0;
        const bool want1 = config_axis2_enabled() &&
                           planner_feed_active_axis(1) &&
                           pio_step_tx_room(1) > 0;
        if (!want0 && !want1) {
          break;
        }
        int emitted = 0;
        if (want0) {
          emitted += planner_fill_fifo(0);
        }
        if (want1) {
          emitted += planner_fill_fifo(1);
        }
        if (emitted == 0) {
          break;
        }
        if (!planner_feed_active()) {
          break;
        }
      }
      if (any_tx_full()) {
        pio_step_arm_tx_irq();
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(2));
      } else {
        pio_step_disarm_tx_irq();
        vTaskDelay(1);
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
  xTaskCreate(task_motion_feed, "feed", 1536, nullptr, configMAX_PRIORITIES - 1,
              &g_feed_handle);
  pio_step_set_feed_task(g_feed_handle);
  xTaskCreate(task_planner, "plan", 2048, nullptr, configMAX_PRIORITIES - 2,
              nullptr);
  xTaskCreate(task_protocol, "proto", 4096, nullptr, tskIDLE_PRIORITY + 2,
              nullptr);
}

#endif /* !HOST_TEST */
