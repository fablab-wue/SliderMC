#include "protocol_internal.h"
#include "motion_api.h"
#include "config_store.h"
#include "board.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

/*
 * Verbose mode (~3 Hz): push compact `#…` status for the UIC to refresh a
 * display (e.g. OLED). Also acts as a heartbeat that the MC is alive.
 * Realtime `?` uses the same line format.
 *
 * Packed channels joined by `|`. A lone idle `0` group is omitted so
 * `| 0 |` becomes `||`.
 */

static char g_last_verbose[512];

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
  if (v != v) {
    snprintf(dst, n, "0");
    return;
  }
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

static void append_str(char *buf, size_t buflen, const char *s) {
  size_t used = strlen(buf);
  size_t sl = strlen(s);
  if (used + sl + 1 >= buflen) {
    return;
  }
  memcpy(buf + used, s, sl + 1);
}

/** Build one axis group into tmp (no leading space). Returns true if non-empty payload. */
static bool format_axis_group(char *tmp, size_t n, float pos, float vel, float acc, float dest,
                              bool moving, bool homing, bool emit_dest) {
  char num[32];
  tmp[0] = 0;
  format_num(num, sizeof(num), pos);
  snprintf(tmp, n, "%s", num);
  if (moving || homing) {
    format_num(num, sizeof(num), fabsf(vel));
    size_t used = strlen(tmp);
    snprintf(tmp + used, n - used, " %s", num);
    format_num(num, sizeof(num), fabsf(acc));
    used = strlen(tmp);
    snprintf(tmp + used, n - used, " %s", num);
    if (moving && !homing && emit_dest) {
      format_num(num, sizeof(num), dest);
      used = strlen(tmp);
      snprintf(tmp + used, n - used, " %s", num);
    }
  }
  return !(tmp[0] == '0' && tmp[1] == 0);
}

/** Build status line into buf (including trailing '\\n'). */
static void build_status_line(char *buf, size_t buflen) {
  McStatus st;
  motion_get_status(&st);
  char letter = protocol_state_letter();
  if (board_camera_ctrl_pulse_active()) {
    letter = 'T';
  } else if (board_camera_ctrl_take_trigger()) {
    letter = 'T';
  }
  snprintf(buf, buflen, "#%c", letter);

  const int nax = config_axis_count();
  const bool emit_dest1 = st.has_target;

  bool prev_keep = false;
  for (int i = 0; i < nax; ++i) {
    bool ch_moving = fabsf(st.vel[i]) >= 0.005f;
    bool ch_homing = st.homing && ch_moving;
    bool emit_dest = (nax >= 2) ? (ch_moving && !st.homing) : (emit_dest1 && ch_moving);
    char grp[64];
    bool keep = format_axis_group(grp, sizeof(grp), st.pos[i], st.vel[i], st.acc[i],
                                  st.target[i], ch_moving, ch_homing, emit_dest);
    if (!keep && nax == 1) {
      keep = true;
    }
    if (i > 0) {
      append_str(buf, buflen, prev_keep ? " |" : "|");
    }
    if (keep) {
      append_str(buf, buflen, " ");
      append_str(buf, buflen, grp);
    }
    prev_keep = keep;
  }
  size_t used = strlen(buf);
  if (used + 1 < buflen) {
    buf[used++] = '\n';
    buf[used] = 0;
  }
}

void protocol_send_verbose(void) {
  char buf[512];
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
  char buf[512];
  build_status_line(buf, sizeof(buf));
  protocol_write(buf);
}
