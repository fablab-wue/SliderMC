#include "board.h"
#include "pins.h"
#include "config_store.h"
#include "debug_hw.h"
#include "axis_hw.h"

#ifndef HOST_TEST

#include <Arduino.h>
#include "hardware/gpio.h"

static const uint8_t k_ext_pins[PIN_EXT_COUNT] = {
    PIN_EXT_1, PIN_EXT_2, PIN_EXT_3, PIN_EXT_4,
};

static bool g_ext_on[PIN_EXT_COUNT];
static bool g_cam_low;
static bool g_cam_consumed;

static bool camera_ctrl_read_low(void) {
#ifdef PIN_CAMERA_CTRL
  return digitalRead(PIN_CAMERA_CTRL) == LOW;
#else
  return false;
#endif
}

bool board_camera_ctrl_take_trigger(void) {
#ifdef PIN_CAMERA_CTRL
  g_cam_low = camera_ctrl_read_low();
#endif
  if (!g_cam_low) {
    g_cam_consumed = false;
    return false;
  }
  if (g_cam_consumed) {
    return false;
  }
  g_cam_consumed = true;
  return true;
}

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
  const int err = axis_hw_error_pin(axis);

  pinMode(step, OUTPUT);
  digitalWrite(step, LOW);
  gpio_set_drive_strength(step, GPIO_DRIVE_STRENGTH_8MA);

  pinMode(dir, OUTPUT);
  digitalWrite(dir, LOW);
  gpio_set_drive_strength(dir, GPIO_DRIVE_STRENGTH_8MA);

  pinMode(err, INPUT_PULLUP);

  if (axis_hw_limit_l_use(axis)) {
    pinMode(axis_hw_limit_l_pin(axis), INPUT_PULLUP);
  }
  if (axis_hw_limit_r_use(axis)) {
    pinMode(axis_hw_limit_r_pin(axis), INPUT_PULLUP);
  }
}

void board_gpio_init(void) {
  (void)config_axis_count();

  int n = axis_hw_count();
  for (int axis = 0; axis < n; ++axis) {
    init_axis_gpio(axis);
  }

  {
    const int en = PIN_DRV_ENABLE;
    pinMode(en, OUTPUT);
    int inactive = axis_hw_en_active(0) ? LOW : HIGH;
    digitalWrite(en, inactive);
    gpio_set_drive_strength(en, GPIO_DRIVE_STRENGTH_12MA);
  }

  for (int i = 0; i < PIN_EXT_COUNT; ++i) {
    if (!config_ext_available(i)) {
      continue;
    }
    pinMode(k_ext_pins[i], OUTPUT);
    gpio_set_drive_strength(k_ext_pins[i], GPIO_DRIVE_STRENGTH_8MA);
    ext_write_level(i, false); /* inactive at boot */
  }

#ifdef PIN_CAMERA_CTRL
  pinMode(PIN_CAMERA_CTRL, INPUT_PULLUP); /* open-collector released */
  gpio_set_drive_strength(PIN_CAMERA_CTRL, GPIO_DRIVE_STRENGTH_12MA);
  g_cam_low = camera_ctrl_read_low();
  g_cam_consumed = g_cam_low; /* held-at-boot does not fire T */
#else
  g_cam_low = false;
  g_cam_consumed = false;
#endif

  board_buzzer_reconfigure();
  dbg_hw_gpio_init();
}

bool board_ext_set(int index, bool on) {
  if (index < 0 || index >= PIN_EXT_COUNT || !config_ext_available(index)) {
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
static bool g_cam_low;
static bool g_cam_consumed;

void board_gpio_init(void) {
  (void)config_axis_count();
  for (int i = 0; i < PIN_EXT_COUNT; ++i) {
    g_ext_on[i] = false;
  }
  g_cam_low = false;
  g_cam_consumed = false;
}

bool board_ext_set(int index, bool on) {
  if (index < 0 || index >= PIN_EXT_COUNT || !config_ext_available(index)) {
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

bool board_camera_ctrl_take_trigger(void) {
  if (!g_cam_low) {
    g_cam_consumed = false;
    return false;
  }
  if (g_cam_consumed) {
    return false;
  }
  g_cam_consumed = true;
  return true;
}

void board_camera_ctrl_inject(bool low) {
  g_cam_low = low;
  if (!low) {
    g_cam_consumed = false;
  }
}

void board_buzzer_pulse(void) {}
void board_buzzer_tick(unsigned dt_ms) { (void)dt_ms; }
void board_buzzer_reconfigure(void) {}

#endif
