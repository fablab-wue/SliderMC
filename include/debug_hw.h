#pragma once
#include "pins.h"
#include "config_store.h"

#if defined(DEBUG_HW) && !defined(HOST_TEST)
#include <Arduino.h>

/* Pico: DBG overlaps LIMIT3 (GP10–11), ERROR 1–3 (GP12–14), ENABLE (GP15).
 * Zero: DBG overlaps EXT_2..4 (GP18–20).
 * ENABLE/ERROR (Pico) and EXT (Zero) are always claimed, so overlapping
 * boards never drive DBG. */
static inline bool dbg_hw_allowed(void) {
#if PIN_DBG_OVERLAPS_DRV || PIN_DBG_OVERLAPS_EXT
  return false;
#else
#if PIN_DBG_OVERLAPS_AXIS2
  if (config_axis2_enabled()) {
    return false;
  }
#endif
#if PIN_DBG_OVERLAPS_AXIS3
  if (config_axis3_enabled()) {
    return false;
  }
#endif
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
