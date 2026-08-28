#include "board.h"
#include "pins.h"
#include "config_store.h"
#include "debug_hw.h"
#include "axis_hw.h"

#ifndef HOST_TEST

#include <Arduino.h>
#include "hardware/gpio.h"

static const uint8_t k_ext_pins[PIN_EXT_COUNT] = {
    PIN_EXT_0, PIN_EXT_1, PIN_EXT_2, PIN_EXT_3,
};

static bool g_ext_on[PIN_EXT_COUNT];

static void ext_write_level(int index, bool on) {
  const McConfig *c = config_get();
  int active = c->ext_active[index] ? 1 : 0;
  int level = on ? active : (active ? 0 : 1);
  digitalWrite(k_ext_pins[index], level ? HIGH : LOW);
  g_ext_on[index] = on;
}

static void init_axis_gpio(int axis) {
  const int step = axis_hw_step_pin(axis);
  const int dir = axis_hw_dir_pin(axis);
  const int en = axis_hw_en_pin(axis);
  const int err = axis_hw_error_pin(axis);

  pinMode(step, OUTPUT);
  digitalWrite(step, LOW);
  gpio_set_drive_strength(step, GPIO_DRIVE_STRENGTH_8MA);

  pinMode(dir, OUTPUT);
  digitalWrite(dir, LOW);
  gpio_set_drive_strength(dir, GPIO_DRIVE_STRENGTH_8MA);

  pinMode(en, OUTPUT);
  {
    int inactive = axis_hw_en_active(axis) ? LOW : HIGH;
    digitalWrite(en, inactive);
  }
  gpio_set_drive_strength(en, GPIO_DRIVE_STRENGTH_8MA);

  pinMode(err, INPUT_PULLUP);

  if (axis_hw_home_use(axis)) {
    pinMode(axis_hw_home_pin(axis), INPUT_PULLUP);
  }
  if (axis_hw_limit_l_use(axis)) {
    pinMode(axis_hw_limit_l_pin(axis), INPUT_PULLUP);
  }
  if (axis_hw_limit_r_use(axis)) {
    pinMode(axis_hw_limit_r_pin(axis), INPUT_PULLUP);
  }
}

void board_gpio_init(void) {
  /* Unsupported boards: clear stray axis2_use before pin setup. */
  (void)config_axis2_enabled();

  init_axis_gpio(0);
  if (config_axis2_enabled()) {
    init_axis_gpio(1);
  }

  for (int i = 0; i < PIN_EXT_COUNT; ++i) {
    pinMode(k_ext_pins[i], OUTPUT);
    ext_write_level(i, false); /* inactive at boot */
  }

  board_buzzer_reconfigure();
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

#define BUZZER_PULSE_MS 100u

static unsigned g_buzzer_remain_ms;
static bool g_buzzer_claimed;

static bool buzzer_hw_ok(void) {
  if (config_get()->buzzer_use == 0) {
    return false;
  }
  if (PIN_BUZZER == PIN_LED) {
    return false;
  }
  return true;
}

void board_buzzer_reconfigure(void) {
  if (!buzzer_hw_ok()) {
    g_buzzer_remain_ms = 0;
    if (g_buzzer_claimed) {
      pinMode(PIN_BUZZER, INPUT);
      g_buzzer_claimed = false;
    }
    return;
  }
  if (!g_buzzer_claimed) {
    pinMode(PIN_BUZZER, OUTPUT);
    digitalWrite(PIN_BUZZER, LOW);
    g_buzzer_claimed = true;
  }
}

void board_buzzer_pulse(void) {
  if (!buzzer_hw_ok()) {
    return;
  }
  if (!g_buzzer_claimed) {
    board_buzzer_reconfigure();
  }
  if (!g_buzzer_claimed) {
    return;
  }
  digitalWrite(PIN_BUZZER, HIGH);
  g_buzzer_remain_ms = BUZZER_PULSE_MS;
}

void board_buzzer_tick(unsigned dt_ms) {
  if (g_buzzer_remain_ms == 0) {
    return;
  }
  if (dt_ms >= g_buzzer_remain_ms) {
    g_buzzer_remain_ms = 0;
    if (g_buzzer_claimed) {
      digitalWrite(PIN_BUZZER, LOW);
    }
    return;
  }
  g_buzzer_remain_ms -= dt_ms;
}

#else /* HOST_TEST */

static bool g_ext_on[PIN_EXT_COUNT];

void board_gpio_init(void) {
  (void)config_axis2_enabled();
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

void board_buzzer_pulse(void) {}
void board_buzzer_tick(unsigned dt_ms) { (void)dt_ms; }
void board_buzzer_reconfigure(void) {}

#endif
