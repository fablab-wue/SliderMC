#pragma once
#include "pins.h"

#if defined(DEBUG_HW) && !defined(HOST_TEST)
#include <Arduino.h>

static inline void dbg_hw_set(int pin, int level) {
  digitalWrite(pin, level ? HIGH : LOW);
}

static inline void dbg_hw_gpio_init(void) {
  const int pins[] = {PIN_DBG_FIFO, PIN_DBG_MOV, PIN_DBG_MOV_CONST, PIN_DBG_CMD,
                      PIN_DBG_IRQ, PIN_DBG_UNDERRUN};
  for (unsigned i = 0; i < sizeof(pins) / sizeof(pins[0]); ++i) {
    pinMode(pins[i], OUTPUT);
    digitalWrite(pins[i], LOW);
  }
}
#else
#define dbg_hw_set(pin, level) ((void)0)
#define dbg_hw_gpio_init() ((void)0)
#endif
