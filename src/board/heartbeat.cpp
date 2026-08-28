#include "board_heartbeat.h"
#include "board.h"
#include "config_store.h"
#include "motion_api.h"
#include "pins.h"

#ifndef HOST_TEST

#include <Arduino.h>

#define HB_WDT_TIMEOUT_MS 2000u

static unsigned g_div;
static unsigned g_counter;
static bool g_unlocked;

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
  "X            XXXXXXXX           X            XXXXXXXX           ",   // LOCKED
};

void board_heartbeat_init(void) {
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);
  g_div = 0;
  g_counter = 0;
  g_unlocked = false;

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
  if (g_unlocked) {
    McStatus st;
    motion_get_status(&st);
    state = (int)st.state;
  }

  digitalWrite(PIN_LED, led_phases[state][g_counter] == 'X' ? HIGH : LOW);
}

void board_heartbeat_tick_old(unsigned dt_ms) {
  (void)dt_ms;

  rp2040.wdt_reset();

  if (++g_div < 3u) {
    return;
  }
  g_div = 0;
  g_counter++;

  bool led;
  if (!g_unlocked) {
    led = !(g_counter & 0x20u); /* ~1 Hz 50% while waiting for first \n */
  } else {
    McStatus st;
    motion_get_status(&st);
    switch (st.state) {
    case MC_STATE_ERROR:
      led = (g_counter & 0x0Cu) != 0; /* ~4 Hz bright */
      break;
    case MC_STATE_HARD_LIMIT:
      led = (g_counter & 0x0Cu) == 0; /* ~4 Hz dark */
      break;
    case MC_STATE_HOMING:
      led = (g_counter & 0x03u) == 0; /* ~16 Hz dark */
      break;
    case MC_STATE_MOVING:
    case MC_STATE_ACCELERATING:
    case MC_STATE_DECELERATING:
      led = (g_counter & 0x03u) != 0; /* ~16 Hz bright */
      break;
    case MC_STATE_DISABLED:
      led = (g_counter & 0x3Fu) == 0; /* ~1 Hz short */
      break;
    case MC_STATE_IDLE:
      led = (g_counter & 0x3Cu) == 0; /* ~1 Hz longer */
      break;
    default:
      led = (g_counter & 0x30u) == 0;
      break;
    }
  }

  digitalWrite(PIN_LED, led ? HIGH : LOW);
}

#else /* HOST_TEST */

void board_heartbeat_init(void) {}
void board_heartbeat_ready(void) {}
void board_heartbeat_tick(unsigned dt_ms) { board_buzzer_tick(dt_ms); }

#endif
