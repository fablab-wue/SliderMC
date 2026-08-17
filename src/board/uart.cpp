#include "board.h"
#include "board_heartbeat.h"
#include "pins.h"
#include "protocol.h"

#include <Arduino.h>

/*
 * UIC link: hardware UART at 1 Mbaud.
 *
 * GP8/GP9 are UART1 pins on the RP2040, which the earlephilhower core exposes
 * as Serial2. setTX()/setRX() live on SerialUART (not the HardwareSerial base)
 * and must be called before begin().
 */

#ifndef HOST_TEST

#include <FreeRTOS.h>
#include <task.h>
#include "hardware/gpio.h"

static SerialUART &UIC = PIN_UART_SERIAL;

void board_uart_init(void) {
  UIC.setTX(PIN_UART_TX);
  UIC.setRX(PIN_UART_RX);
  UIC.begin(UART_BAUD);
  gpio_set_drive_strength(PIN_UART_TX, GPIO_DRIVE_STRENGTH_8MA);
}

void board_wait_unlock_newline(void) {
  for (;;) {
    while (UIC.available() > 0) {
      if ((uint8_t)UIC.read() == (uint8_t)'\n') {
        return;
      }
    }
    while (Serial.available() > 0) {
      if ((uint8_t)Serial.read() == (uint8_t)'\n') {
        return;
      }
    }
    board_heartbeat_tick(5);
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void board_uart_poll_rx(void) {
  while (UIC.available() > 0) {
    protocol_feed_uart_byte((uint8_t)UIC.read());
  }
}

void board_uart_write(const char *data, size_t n) {
  if (!data || n == 0) {
    return;
  }
  UIC.write(reinterpret_cast<const uint8_t *>(data), n);
}

#endif
