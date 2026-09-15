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
static BoardExtMode g_ext_mode[PIN_EXT_COUNT];
static bool g_cam_low;
static bool g_cam_consumed;
static unsigned g_cam_pulse_ms;

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

bool board_camera_ctrl_pulse_active(void) { return g_cam_pulse_ms > 0; }

void board_camera_ctrl_pulse(unsigned ms) {
#ifdef PIN_CAMERA_CTRL
  if (ms == 0) {
    return;
  }
  pinMode(PIN_CAMERA_CTRL, OUTPUT);
  digitalWrite(PIN_CAMERA_CTRL, LOW);
  g_cam_pulse_ms = ms;
#else
  (void)ms;
#endif
}

void board_camera_ctrl_tick(unsigned dt_ms) {
  if (g_cam_pulse_ms == 0) {
    return;
  }
  if (dt_ms >= g_cam_pulse_ms) {
    g_cam_pulse_ms = 0;
#ifdef PIN_CAMERA_CTRL
    pinMode(PIN_CAMERA_CTRL, INPUT_PULLUP);
#endif
    return;
  }
  g_cam_pulse_ms -= dt_ms;
}

static void ext_apply_pin(int index) {
  const McConfig *c = config_get();
  int active = c->ext_active[index] ? 1 : 0;
  int level = g_ext_on[index] ? active : (active ? 0 : 1);
  uint8_t pin = k_ext_pins[index];
  switch (g_ext_mode[index]) {
  case BOARD_EXT_IN:
    pinMode(pin, INPUT_PULLUP);
    break;
  case BOARD_EXT_OUT:
    pinMode(pin, OUTPUT);
    gpio_set_drive_strength(pin, GPIO_DRIVE_STRENGTH_8MA);
    digitalWrite(pin, level ? HIGH : LOW);
    break;
  case BOARD_EXT_OC:
    if (g_ext_on[index]) {
      pinMode(pin, OUTPUT);
      gpio_set_drive_strength(pin, GPIO_DRIVE_STRENGTH_8MA);
      digitalWrite(pin, level ? HIGH : LOW);
    } else {
      pinMode(pin, INPUT_PULLUP);
    }
    break;
  }
}

static void ext_write_level(int index, bool on) {
  g_ext_on[index] = on;
  if (g_ext_mode[index] == BOARD_EXT_IN) {
    g_ext_mode[index] = BOARD_EXT_OUT;
  }
  ext_apply_pin(index);
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
    g_ext_on[i] = false;
    g_ext_mode[i] = BOARD_EXT_IN;
    if (!config_ext_available(i)) {
      continue;
    }
    ext_apply_pin(i);
  }

#ifdef PIN_CAMERA_CTRL
  pinMode(PIN_CAMERA_CTRL, INPUT_PULLUP); /* open-collector released */
  gpio_set_drive_strength(PIN_CAMERA_CTRL, GPIO_DRIVE_STRENGTH_12MA);
  g_cam_low = camera_ctrl_read_low();
  g_cam_consumed = g_cam_low; /* held-at-boot does not fire T */
  g_cam_pulse_ms = 0;
#else
  g_cam_low = false;
  g_cam_consumed = false;
  g_cam_pulse_ms = 0;
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

bool board_ext_set_mode(int index, BoardExtMode mode) {
  if (index < 0 || index >= PIN_EXT_COUNT || !config_ext_available(index)) {
    return false;
  }
  g_ext_mode[index] = mode;
  ext_apply_pin(index);
  return true;
}

BoardExtMode board_ext_get_mode(int index) {
  if (index < 0 || index >= PIN_EXT_COUNT) {
    return BOARD_EXT_IN;
  }
  return g_ext_mode[index];
}

bool board_ext_read_pin(int index, int *high) {
  if (index < 0 || index >= PIN_EXT_COUNT || !config_ext_available(index) || !high) {
    return false;
  }
  *high = digitalRead(k_ext_pins[index]) == HIGH ? 1 : 0;
  return true;
}

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

void board_buzzer_pulse(unsigned ms) {
  if (!buzzer_hw_ok() || ms == 0) {
    return;
  }
  if (!g_buzzer_claimed) {
    board_buzzer_reconfigure();
  }
  if (!g_buzzer_claimed) {
    return;
  }
  digitalWrite(PIN_BUZZER, HIGH);
  g_buzzer_remain_ms = ms;
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
static BoardExtMode g_ext_mode[PIN_EXT_COUNT];
static bool g_cam_low;
static bool g_cam_consumed;
static unsigned g_cam_pulse_ms;

void board_gpio_init(void) {
  (void)config_axis_count();
  for (int i = 0; i < PIN_EXT_COUNT; ++i) {
    g_ext_on[i] = false;
    g_ext_mode[i] = BOARD_EXT_IN;
  }
  g_cam_low = false;
  g_cam_consumed = false;
  g_cam_pulse_ms = 0;
}

bool board_ext_set(int index, bool on) {
  if (index < 0 || index >= PIN_EXT_COUNT || !config_ext_available(index)) {
    return false;
  }
  g_ext_on[index] = on;
  if (g_ext_mode[index] == BOARD_EXT_IN) {
    g_ext_mode[index] = BOARD_EXT_OUT;
  }
  return true;
}

bool board_ext_get(int index) {
  if (index < 0 || index >= PIN_EXT_COUNT) {
    return false;
  }
  return g_ext_on[index];
}

bool board_ext_set_mode(int index, BoardExtMode mode) {
  if (index < 0 || index >= PIN_EXT_COUNT || !config_ext_available(index)) {
    return false;
  }
  g_ext_mode[index] = mode;
  return true;
}

BoardExtMode board_ext_get_mode(int index) {
  if (index < 0 || index >= PIN_EXT_COUNT) {
    return BOARD_EXT_IN;
  }
  return g_ext_mode[index];
}

bool board_ext_read_pin(int index, int *high) {
  if (index < 0 || index >= PIN_EXT_COUNT || !config_ext_available(index) || !high) {
    return false;
  }
  *high = g_ext_on[index] ? 1 : 0;
  return true;
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

bool board_camera_ctrl_pulse_active(void) { return g_cam_pulse_ms > 0; }

void board_camera_ctrl_pulse(unsigned ms) {
  if (ms == 0) {
    return;
  }
  g_cam_pulse_ms = ms;
  g_cam_low = true;
}

void board_camera_ctrl_tick(unsigned dt_ms) {
  if (g_cam_pulse_ms == 0) {
    return;
  }
  if (dt_ms >= g_cam_pulse_ms) {
    g_cam_pulse_ms = 0;
    g_cam_low = false;
    g_cam_consumed = false;
    return;
  }
  g_cam_pulse_ms -= dt_ms;
}

void board_buzzer_pulse(unsigned ms) { (void)ms; }
void board_buzzer_tick(unsigned dt_ms) { (void)dt_ms; }
void board_buzzer_reconfigure(void) {}

#endif
