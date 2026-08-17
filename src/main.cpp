#include "board.h"
#include "board_config_fs.h"
#include "motion_task.h"
#include "motion_api.h"
#include "config_store.h"

#ifndef HOST_TEST

#include <Arduino.h>

/*
 * SliderMC firmware entry.
 * FreeRTOS tasks own motion feed, planner, and protocol I/O.
 * LED heartbeat + optional WDT run from the protocol task (not loop).
 */

void setup() {
  Serial.begin(115200);
  delay(200);

  config_init_defaults();
  (void)board_config_load_from_fs();
  session_sync_from_config();

  board_gpio_init(); /* DRV_ERROR / limits ready before first planner tick */
  motion_init();     /* PIO + planner before feed/plan tasks run */
  board_uart_init();

  motion_tasks_start();
}

void loop() {
  /* Idle — never block STEP feed. Heartbeat lives in task_protocol. */
  delay(100);
}

#endif
