#include "servo_pwm.h"
#include "config_store.h"
#include "config_defaults.h"
#include "pins.h"

#include <math.h>
#include <stdint.h>

#ifndef HOST_TEST
#include <Arduino.h>
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#endif

#define SERVO_FRAME_HZ 100u
#define SERVO_WRAP 65535u
#define SERVO_US_MIN 1000.0f
#define SERVO_US_MAX 2000.0f

static bool g_pwm_on;
static float g_last_deg[SERVO_MAX];

static float servo_boot_deg(int s) {
  const McConfig *c = config_get();
  float mn = c->servo_min[s];
  float mx = c->servo_max[s];
  float p = 0.0f;
  if (!isnan(mn) && p < mn) {
    p = mn;
  }
  if (!isnan(mx) && p > mx) {
    p = mx;
  }
  return p;
}

#ifndef HOST_TEST

static int servo_pin(int s) {
  if (s == 1) {
    return PIN_SERVO_2;
  }
  if (s == 2) {
    return PIN_SERVO_3;
  }
  return PIN_SERVO_1;
}

static uint16_t deg_to_cc(int s, float deg) {
  const McConfig *c = config_get();
  float mn = c->servo_min[s];
  float mx = c->servo_max[s];
  if (isnan(mn)) {
    mn = CFG_DEFAULT_SERVO_MIN;
  }
  if (isnan(mx)) {
    mx = CFG_DEFAULT_SERVO_MAX;
  }
  if (mx < mn) {
    float t = mn;
    mn = mx;
    mx = t;
  }
  if (deg < mn) {
    deg = mn;
  }
  if (deg > mx) {
    deg = mx;
  }
  float span = mx - mn;
  float us = SERVO_US_MIN;
  if (span > 1e-6f) {
    us = SERVO_US_MIN + (deg - mn) / span * (SERVO_US_MAX - SERVO_US_MIN);
  }
  if (us < SERVO_US_MIN) {
    us = SERVO_US_MIN;
  }
  if (us > SERVO_US_MAX) {
    us = SERVO_US_MAX;
  }
  float cc = us * ((float)(SERVO_WRAP + 1u) / 10000.0f);
  if (cc < 0.0f) {
    cc = 0.0f;
  }
  if (cc > (float)SERVO_WRAP) {
    cc = (float)SERVO_WRAP;
  }
  return (uint16_t)(cc + 0.5f);
}

static void apply_cc(int s, uint16_t cc) {
  int pin = servo_pin(s);
  uint slice = pwm_gpio_to_slice_num((uint)pin);
  uint chan = pwm_gpio_to_channel((uint)pin);
  pwm_set_chan_level(slice, chan, cc);
}

static void setup_slices(int ns) {
  uint32_t sys = clock_get_hz(clk_sys);
  float div = (float)sys / ((float)SERVO_FRAME_HZ * (float)(SERVO_WRAP + 1u));

  bool slice_done[8] = {false};
  for (int s = 0; s < ns; ++s) {
    uint slice = pwm_gpio_to_slice_num((uint)servo_pin(s));
    if (slice >= 8u || slice_done[slice]) {
      continue;
    }
    slice_done[slice] = true;
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, div);
    pwm_config_set_wrap(&cfg, SERVO_WRAP);
    pwm_init(slice, &cfg, false);
  }

  for (int s = 0; s < ns; ++s) {
    int pin = servo_pin(s);
    gpio_set_function((uint)pin, GPIO_FUNC_PWM);
    gpio_set_drive_strength((uint)pin, GPIO_DRIVE_STRENGTH_8MA);
    apply_cc(s, 0);
  }

  bool inv_a[8] = {false};
  bool inv_b[8] = {false};
  for (int s = 0; s < ns; ++s) {
    uint slice = pwm_gpio_to_slice_num((uint)servo_pin(s));
    uint chan = pwm_gpio_to_channel((uint)servo_pin(s));
    bool inv = (config_get()->servo_active[s] == 0);
    if (slice < 8u) {
      if (chan == PWM_CHAN_A) {
        inv_a[slice] = inv;
      } else {
        inv_b[slice] = inv;
      }
    }
  }
  for (uint sl = 0; sl < 8u; ++sl) {
    if (slice_done[sl]) {
      pwm_set_output_polarity(sl, inv_a[sl], inv_b[sl]);
    }
  }
}

#endif /* HOST_TEST */

void servo_pwm_init(void) {
  int ns = config_servo_count();
  for (int s = 0; s < SERVO_MAX; ++s) {
    g_last_deg[s] = servo_boot_deg(s);
  }
#ifndef HOST_TEST
  setup_slices(ns);
  for (int s = 0; s < ns; ++s) {
    apply_cc(s, deg_to_cc(s, g_last_deg[s]));
  }
#endif
  g_pwm_on = false;
}

void servo_pwm_reconfigure(void) { servo_pwm_init(); }

void servo_pwm_set_enabled(bool on) {
  int ns = config_servo_count();
  g_pwm_on = on && ns > 0;
#ifndef HOST_TEST
  bool slice_done[8] = {false};
  for (int s = 0; s < ns; ++s) {
    uint slice = pwm_gpio_to_slice_num((uint)servo_pin(s));
    if (g_pwm_on) {
      apply_cc(s, deg_to_cc(s, g_last_deg[s]));
    } else {
      apply_cc(s, 0);
    }
    if (slice < 8u && !slice_done[slice]) {
      slice_done[slice] = true;
      pwm_set_enabled(slice, g_pwm_on);
    }
  }
#endif
}

bool servo_pwm_enabled(void) { return g_pwm_on; }

void servo_pwm_write_deg(int servo0, float deg) {
  if (servo0 < 0 || servo0 >= config_servo_count()) {
    return;
  }
  g_last_deg[servo0] = deg;
  if (!g_pwm_on) {
    return;
  }
#ifndef HOST_TEST
  apply_cc(servo0, deg_to_cc(servo0, deg));
#endif
}
