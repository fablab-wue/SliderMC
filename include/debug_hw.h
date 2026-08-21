#pragma once
#include "pins.h"
#include "config_store.h"

#if defined(DEBUG_HW) && !defined(HOST_TEST)
#include <Arduino.h>

/* On Pico, DBG GPIOs overlap axis2 (GP10–13) — never drive them when axis2 on.
 * On Zero, DBG and axis2 are disjoint; both may be active together. */
static inline bool dbg_hw_allowed(void) {
#if PIN_DBG_OVERLAPS_AXIS2
  return !config_axis2_enabled();
#else
  return true;
#endif
}

static inline void dbg_hw_set(int pin, int level) {
  if (!dbg_hw_allowed()) {
    return;
  }
  digitalWrite(pin, level ? HIGH : LOW);
}

static inline void dbg_hw_gpio_init(void) {
  if (!dbg_hw_allowed()) {
    return;
  }
  const int pins[] = {PIN_DBG_FIFO, PIN_DBG_MOV, PIN_DBG_MOV_CONST, PIN_DBG_CMD,
                      PIN_DBG_IRQ, PIN_DBG_UNDERRUN};
  for (unsigned i = 0; i < sizeof(pins) / sizeof(pins[0]); ++i) {
    pinMode(pins[i], OUTPUT);
    digitalWrite(pins[i], LOW);
  }
}
#else
#define dbg_hw_allowed() (true)
#define dbg_hw_set(pin, level) ((void)0)
#define dbg_hw_gpio_init() ((void)0)
#endif
