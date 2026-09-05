#include "protocol_internal.h"
#include "motion_api.h"
#include "config_store.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

/*
 * Verbose mode (~3 Hz): push compact `#…` status for the UIC to refresh a
 * display (e.g. OLED). Also acts as a heartbeat that the MC is alive.
 * Realtime `?` uses the same line format.
 *
 * Each axis is a 1-axis group. 2-axis joins groups with " | ".
 *
 * 1-axis:  #I pos
 *          #M pos speed accel [target]
 *          #H pos speed accel
 * 2-axis:  #I pos1 | pos2
 *          #H pos1 speed1 accel1 | pos2 speed2 accel2
 *          #M pos1 speed1 accel1 target1 | pos2 speed2 accel2 target2
 */

static char g_last_verbose[192];

void protocol_verbose_reset_dedupe(void) { g_last_verbose[0] = 0; }

char protocol_state_letter(void) {
  McStatus st;
  motion_get_status(&st);
  switch (st.state) {
  case MC_STATE_ERROR:
    return 'E';
  case MC_STATE_HARD_LIMIT:
    return 'L';
  case MC_STATE_DISABLED:
    return 'D';
  case MC_STATE_HOMING:
    return 'H';
  case MC_STATE_PATH:
    return 'P';
  case MC_STATE_ACCELERATING:
    return 'A';
  case MC_STATE_DECELERATING:
    return 'B';
  case MC_STATE_MOVING:
    return 'M';
  case MC_STATE_IDLE:
  default:
    return 'I';
  }
}

/** Format with at most 2 decimals (truncate, not round); strip trailing zeros. */
static void format_num(char *dst, size_t n, float v) {
  if (n == 0) {
    return;
  }
  /* Reject NaN. */
  if (v != v) {
    snprintf(dst, n, "0");
    return;
  }
  /* Truncate toward zero to 2 decimals: 100.999… → 100.99 */
  float scaled = v * 100.0f;
  float trunc_scaled = (scaled >= 0.0f) ? floorf(scaled) : ceilf(scaled);
  float t = trunc_scaled / 100.0f;
  if (fabsf(t) < 0.005f) {
    snprintf(dst, n, "0");
    return;
  }
  char tmp[32];
  snprintf(tmp, sizeof(tmp), "%.2f", (double)t);
  size_t len = strlen(tmp);
  while (len > 0 && tmp[len - 1] == '0') {
    tmp[--len] = 0;
  }
  if (len > 0 && tmp[len - 1] == '.') {
    tmp[--len] = 0;
  }
  if (len == 0 || (len == 1 && tmp[0] == '-')) {
    snprintf(dst, n, "0");
    return;
  }

  snprintf(dst, n, "%s", tmp);
}

static void append_num(char *buf, size_t buflen, float v) {
  char num[32];
  format_num(num, sizeof(num), v);
  size_t used = strlen(buf);
  if (used + 1 + strlen(num) + 1 >= buflen) {
    return;
  }
  buf[used++] = ' ';
  buf[used] = 0;
  strncat(buf, num, buflen - used - 1);
}

/** Append one axis group: pos [speed accel [target]]. */
static void append_axis_group(char *buf, size_t buflen, float pos, float vel, float acc,
                              float dest, bool moving, bool homing, bool emit_dest) {
  append_num(buf, buflen, pos);
  if (moving || homing) {
    append_num(buf, buflen, fabsf(vel));
    append_num(buf, buflen, fabsf(acc));
    if (moving && !homing && emit_dest) {
      append_num(buf, buflen, dest);
    }
  }
}

/** Build status line into buf (including trailing '\\n'). */
static void build_status_line(char *buf, size_t buflen) {
  McStatus st;
  motion_get_status(&st);
  char letter = protocol_state_letter();
  snprintf(buf, buflen, "#%c", letter);

  const bool ax2 = config_axis2_enabled();
  const bool emit_dest1 = st.has_target;
  const bool emit_dest2 = st.moving && !st.homing;

  append_axis_group(buf, buflen, st.pos_mm, st.vel_mm_s, st.acc_mm_s2, st.target_mm,
                    st.moving, st.homing, ax2 ? emit_dest2 : emit_dest1);
  if (ax2) {
    size_t used = strlen(buf);
    if (used + 3 < buflen) {
      buf[used++] = ' ';
      buf[used++] = '|';
      buf[used] = 0;
    }
    append_axis_group(buf, buflen, st.pos_mm_2, st.vel_mm_s_2, st.acc_mm_s2_2,
                      st.target_mm_2, st.moving, st.homing, emit_dest2);
  }
  size_t used = strlen(buf);
  if (used + 1 < buflen) {
    buf[used++] = '\n';
    buf[used] = 0;
  }
}

void protocol_send_verbose(void) {
  char buf[192];
  build_status_line(buf, sizeof(buf));

  if (session_get()->terminal) {
    if (strcmp(buf, g_last_verbose) == 0) {
      return;
    }
    strncpy(g_last_verbose, buf, sizeof(g_last_verbose) - 1);
    g_last_verbose[sizeof(g_last_verbose) - 1] = 0;
  }

  protocol_write(buf);
}

void protocol_send_status(void) {
  char buf[192];
  build_status_line(buf, sizeof(buf));
  protocol_write(buf);
}
