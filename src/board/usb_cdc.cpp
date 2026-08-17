#include "board.h"

#include <Arduino.h>

#ifndef HOST_TEST

void board_usb_write(const char *data, size_t n) {
  if (!data || n == 0) {
    return;
  }
  Serial.write(reinterpret_cast<const uint8_t *>(data), n);
}

void board_usb_debug(const char *data, size_t n) {
  /* Debug is USB-only by design (never board_uart_write). */
  board_usb_write(data, n);
}

bool board_usb_connected(void) {
#if defined(ARDUINO_ARCH_RP2040)
  return Serial;
#else
  return true;
#endif
}

#endif
