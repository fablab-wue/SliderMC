#include "board_heartbeat.h"
#include "board.h"
#include "config_store.h"
#include "motion_api.h"
#include "motion_diag.h"
#include "pins.h"
#include "status_neopixel.h"

#ifndef HOST_TEST

#include <Arduino.h>

#define HB_WDT_TIMEOUT_MS 2000u
#define HB_RAINBOW_MS 1000u
#define HB_UNDERRUN_BLINK_MS 80u
#define HB_HARD_BLINK_MS 80u
#define HB_HOME_BLINK_MS 250u

static unsigned g_div;
static unsigned g_counter;
static bool g_unlocked;
static uint32_t g_boot_ms;

// Status LED blinking/flashing phases - keep syncrone with typedef enum McState;
static char led_phases[][65] = {
//           1         2         3         4         5         6
// 0123456789012345678901234567890123456789012345678901234567890123
  "X   X   X                                                       ",   // DISABLED
  "X                                                               ",   // IDLE
  "X          XX         XXX        XXXX       XXXXX      XXXXXX   ",   // ACCELERATING
  "XXXXXX     XXXXXX     XXXXXX    XXXXXX     XXXXXX     XXXXXX    ",   // MOVING
  "XXXXXX      XXXXX       XXXX        XXX         XX          X   ",   // DECELERATING
  "XXXXX  XXXX   XXX    XX    X    XXXXX  XXXX   XXX    XX    X    ",   // HOMING
  "XXXX       XXXX       XXXX                                      ",   // HARD_LIMIT
  "XXXX       XXXX       XXXXXXXXXXXXXXXX                          ",   // ERROR
  "X   X   X   X   X   X   X   X   X   X   X   X   X   X   X   X   ",   // PATH
  "X            XXXXXXXX           X            XXXXXXXX           ",   // LOCKED
};

static void color_wheel(uint8_t pos, uint8_t *r, uint8_t *g, uint8_t *b) {
  pos = (uint8_t)(255u - pos);
  if (pos < 85u) {
    *r = (uint8_t)(255u - pos * 3u);
    *g = 0;
    *b = (uint8_t)(pos * 3u);
  } else if (pos < 170u) {
    pos = (uint8_t)(pos - 85u);
    *r = 0;
    *g = (uint8_t)(pos * 3u);
    *b = (uint8_t)(255u - pos * 3u);
  } else {
    pos = (uint8_t)(pos - 170u);
    *r = (uint8_t)(pos * 3u);
    *g = (uint8_t)(255u - pos * 3u);
    *b = 0;
  }
  /* JKSlider rainbow v=0.55 */
  *r = (uint8_t)(((unsigned)*r * 140u) / 255u);
  *g = (uint8_t)(((unsigned)*g * 140u) / 255u);
  *b = (uint8_t)(((unsigned)*b * 140u) / 255u);
}

static bool blink_on(uint32_t now_ms, unsigned half_ms) {
  return ((now_ms / half_ms) & 1u) != 0;
}

static void neopixel_from_status(int state, const McStatus *st) {
  uint8_t r = 0;
  uint8_t g = 0;
  uint8_t b = 0;
  uint32_t now = millis();

  if (!g_unlocked) {
    uint32_t elapsed = now - g_boot_ms;
    uint8_t hue = (uint8_t)(((elapsed % HB_RAINBOW_MS) * 255u) / HB_RAINBOW_MS);
    color_wheel(hue, &r, &g, &b);
  } else if ((st && st->drv_error) || state == MC_STATE_ERROR) {
    r = 255;
  } else if (motion_diag_underrun_latched()) {
    if (blink_on(now, HB_UNDERRUN_BLINK_MS)) {
      r = 255;
    }
  } else if (state == MC_STATE_HARD_LIMIT) {
    if (blink_on(now, HB_HARD_BLINK_MS)) {
      r = 255;
    }
  } else if (state == MC_STATE_HOMING) {
    if (blink_on(now, HB_HOME_BLINK_MS)) {
      r = 255;
    }
  } else if (state == MC_STATE_PATH) {
    g = 200;
    b = 200; /* path play: cyan */
  } else if (state == MC_STATE_ACCELERATING || state == MC_STATE_DECELERATING) {
    r = 255;
    g = 255; /* yellow */
  } else if (state == MC_STATE_MOVING) {
    g = 255; /* cruise green */
  } else if (state == MC_STATE_DISABLED) {
    r = 31;
    g = 11; /* dim orange */
  } else if (state == MC_STATE_IDLE) {
    if (board_uic_linked()) {
      r = 31;
      g = 31;
      b = 31; /* UIC linked: dim white */
    } else {
      r = 31;
      b = 31; /* USB-only / no UIC: dim purple */
    }
  } else {
    r = 31;
    g = 11; /* dim orange */
  }

  status_neopixel_set(r, g, b);
}

void board_heartbeat_init(void) {
  status_neopixel_begin();
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);
  g_div = 0;
  g_counter = 0;
  g_unlocked = false;
  g_boot_ms = millis();

  if (config_get()->wdt_use) {
    rp2040.wdt_begin(HB_WDT_TIMEOUT_MS);
  }
}

void board_heartbeat_ready(void) {
  g_unlocked = true;
}

void board_heartbeat_tick(unsigned dt_ms) {
  board_buzzer_tick(dt_ms);

  rp2040.wdt_reset();

  if (++g_div < 3u) {
    return;
  }
  g_div = 0;

  g_counter = (++g_counter) & 0x3F;

  int state = MC_STATE_LOCKED;
  McStatus st;
  if (g_unlocked) {
    motion_get_status(&st);
    state = (int)st.state;
    neopixel_from_status(state, &st);
  } else {
    neopixel_from_status(state, nullptr);
  }

  digitalWrite(PIN_LED, led_phases[state][g_counter] == 'X' ? HIGH : LOW);
}

#else /* HOST_TEST */

void board_heartbeat_init(void) {}
void board_heartbeat_ready(void) {}
void board_heartbeat_tick(unsigned dt_ms) { board_buzzer_tick(dt_ms); }

#endif
