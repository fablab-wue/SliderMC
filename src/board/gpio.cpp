#include "board.h"
#include "pins.h"
#include "config_store.h"
#include "debug_hw.h"

#ifndef HOST_TEST

#include <Arduino.h>
#include "hardware/gpio.h"

static const uint8_t k_ext_pins[PIN_EXT_COUNT] = {
    PIN_EXT_0, PIN_EXT_1, PIN_EXT_2, PIN_EXT_3, PIN_EXT_4,
    PIN_EXT_5, PIN_EXT_6, PIN_EXT_7, PIN_EXT_8, PIN_EXT_9,
};

static bool g_ext_on[PIN_EXT_COUNT];

static void ext_write_level(int index, bool on) {
  const McConfig *c = config_get();
  int active = c->ext_active[index] ? 1 : 0;
  int level = on ? active : (active ? 0 : 1);
  digitalWrite(k_ext_pins[index], level ? HIGH : LOW);
  g_ext_on[index] = on;
}

void board_gpio_init(void) {
  pinMode(PIN_DRV_STEP, OUTPUT);
  digitalWrite(PIN_DRV_STEP, LOW);
  gpio_set_drive_strength(PIN_DRV_STEP, GPIO_DRIVE_STRENGTH_8MA);

  pinMode(PIN_DRV_DIR, OUTPUT);
  digitalWrite(PIN_DRV_DIR, LOW);
  gpio_set_drive_strength(PIN_DRV_DIR, GPIO_DRIVE_STRENGTH_8MA);

  pinMode(PIN_DRV_EN, OUTPUT);
  /* Inactive until SE enables: opposite of DRV_EN_active. */
  {
    const McConfig *c = config_get();
    int inactive = c->drv_en_active ? LOW : HIGH;
    digitalWrite(PIN_DRV_EN, inactive);
  }
  gpio_set_drive_strength(PIN_DRV_EN, GPIO_DRIVE_STRENGTH_8MA);

  pinMode(PIN_DRV_ERROR, INPUT_PULLUP);

  const McConfig *c = config_get();
  if (c->sw_home_use) {
    pinMode(PIN_SW_HOME, INPUT_PULLUP);
  }
  if (c->sw_limit_l_use) {
    pinMode(PIN_SW_LIMIT_L, INPUT_PULLUP);
  }
  if (c->sw_limit_r_use) {
    pinMode(PIN_SW_LIMIT_R, INPUT_PULLUP);
  }

  for (int i = 0; i < PIN_EXT_COUNT; ++i) {
    pinMode(k_ext_pins[i], OUTPUT);
    ext_write_level(i, false); /* inactive at boot */
  }

  dbg_hw_gpio_init();
}

bool board_ext_set(int index, bool on) {
  if (index < 0 || index >= PIN_EXT_COUNT) {
    return false;
  }
  ext_write_level(index, on);
  return true;
}

bool board_ext_get(int index) {
  if (index < 0 || index >= PIN_EXT_COUNT) {
    return false;
  }
  return g_ext_on[index];
}

#else /* HOST_TEST */

static bool g_ext_on[PIN_EXT_COUNT];

void board_gpio_init(void) {
  for (int i = 0; i < PIN_EXT_COUNT; ++i) {
    g_ext_on[i] = false;
  }
}

bool board_ext_set(int index, bool on) {
  if (index < 0 || index >= PIN_EXT_COUNT) {
    return false;
  }
  g_ext_on[index] = on;
  return true;
}

bool board_ext_get(int index) {
  if (index < 0 || index >= PIN_EXT_COUNT) {
    return false;
  }
  return g_ext_on[index];
}

#endif
