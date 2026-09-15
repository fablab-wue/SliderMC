#include "protocol_internal.h"
#include "config_store.h"
#include "board_config_fs.h"
#include "board.h"
#include "motion_api.h"
#include "motion_diag.h"
#include "motion_path.h"
#include "config_defaults.h"
#include "pins.h"
#include "debug_hw.h"
#include "version.h"
#ifndef HOST_TEST
#include "pio_step.h"
#endif
#include "servo_pwm.h"

#include <ctype.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef HOST_TEST
#include <Arduino.h>
#endif

static int icmp_prefix(const char *s, const char *verb) {
  size_t n = strlen(verb);
  for (size_t i = 0; i < n; ++i) {
    char ca = s[i];
    char cb = verb[i];
    if (ca >= 'A' && ca <= 'Z') {
      ca = (char)(ca - 'A' + 'a');
    }
    if (cb >= 'A' && cb <= 'Z') {
      cb = (char)(cb - 'A' + 'a');
    }
    if (ca != cb) {
      return (int)(unsigned char)ca - (int)(unsigned char)cb;
    }
  }
  return 0;
}

/** Config-key prefix: next char must not be a letter (`axis` vs `axis2`). */
static bool starts_cmd(const char *s, const char *verb, const char **rest) {
  size_t n = strlen(verb);
  if (icmp_prefix(s, verb) != 0) {
    return false;
  }
  char next = s[n];
  if (next != 0 && isalpha((unsigned char)next)) {
    return false;
  }
  const char *p = s + n;
  while (*p == ' ' || *p == '\t') {
    ++p;
  }
  *rest = p;
  return true;
}

/** Two-letter verb; remainder may start with X/Y/Z, a digit, `_`, or `none`. */
static bool cmd_verb(const char *s, const char *verb, const char **rest) {
  size_t n = strlen(verb);
  if (n != 2 || icmp_prefix(s, verb) != 0) {
    return false;
  }
  const char *p = s + n;
  while (*p == ' ' || *p == '\t') {
    ++p;
  }
  *rest = p;
  return true;
}

static bool match_any(const char *s, const char **rest, const char *a, const char *b,
                      const char *c) {
  (void)b;
  (void)c;
  return cmd_verb(s, a, rest);
}

static bool parse_float_arg(const char *s, float *out) {
  if (!s || !*s) {
    return false;
  }
  char *end = nullptr;
  float v = strtof(s, &end);
  if (end == s) {
    return false;
  }
  while (*end == ' ' || *end == '\t') {
    ++end;
  }
  if (*end != 0) {
    return false;
  }
  *out = v;
  return true;
}

static void skip_ws(const char **s) {
  while (**s == ' ' || **s == '\t') {
    ++*s;
  }
}

static int axis_letter(char c) {
  int nm = config_motor_count();
  int ns = config_servo_count();
  if (c == 'x' || c == 'X') {
    return (nm >= 1) ? 0 : -1;
  }
  if (c == 'y' || c == 'Y') {
    return (nm >= 2) ? 1 : -1;
  }
  if (c == 'z' || c == 'Z') {
    return (nm >= 3) ? 2 : -1;
  }
  if (c == 'a' || c == 'A') {
    return (ns >= 1) ? nm : -1;
  }
  if (c == 'b' || c == 'B') {
    return (ns >= 2) ? (nm + 1) : -1;
  }
  if (c == 'c' || c == 'C') {
    return (ns >= 3) ? (nm + 2) : -1;
  }
  return -1;
}

/** `none` token; next must be end, whitespace, or an axis letter (glued `SLZnoneY40`). */
static bool take_none(const char **s) {
  const char *p = *s;
  if (!((p[0] == 'n' || p[0] == 'N') && (p[1] == 'o' || p[1] == 'O') &&
        (p[2] == 'n' || p[2] == 'N') && (p[3] == 'e' || p[3] == 'E'))) {
    return false;
  }
  char n = p[4];
  if (n != 0 && n != ' ' && n != '\t' && axis_letter(n) < 0) {
    return false;
  }
  *s = p + 4;
  return true;
}

static bool take_float(const char **s, float *out) {
  char *end = nullptr;
  float v = strtof(*s, &end);
  if (end == *s) {
    return false;
  }
  *out = v;
  *s = end;
  return true;
}

static bool take_int(const char **s, int *out) {
  char *end = nullptr;
  long v = strtol(*s, &end, 10);
  if (end == *s) {
    return false;
  }
  *out = (int)v;
  *s = end;
  return true;
}

static bool parse_optional_pulse_ms(const char *s, int lo, int hi, int *out) {
  skip_ws(&s);
  if (*s == 0) {
    *out = 100;
    return true;
  }
  int v = 0;
  if (!take_int(&s, &v)) {
    return false;
  }
  skip_ws(&s);
  if (*s != 0) {
    return false;
  }
  if (v < lo || v > hi) {
    return false;
  }
  *out = v;
  return true;
}

enum AxisArgKind { AXIS_MOVE = 0, AXIS_WINDOW, AXIS_PATH, AXIS_JOY };

typedef struct {
  float v[MC_CH_MAX];
  bool set[MC_CH_MAX];
  bool named;
} AxisParsed;

static bool axis_fitted(int ax) { return ax >= 0 && ax < config_axis_count(); }

static void axis_parsed_clear(AxisParsed *out) {
  for (int i = 0; i < MC_CH_MAX; ++i) {
    out->v[i] = NAN;
    out->set[i] = false;
  }
  out->named = false;
}

static bool any_set(const AxisParsed *p) {
  for (int i = 0; i < MC_CH_MAX; ++i) {
    if (p->set[i]) {
      return true;
    }
  }
  return false;
}

/**
 * Dual syntax: positional (`100`, `_ _ 40`) or named (`Z100`, `X20A-45`). Not mixed.
 * MOVE: `_` → NAN. WINDOW: `_` skip, `none` → set+NAN. PATH: `_`/omit → 0 (int).
 * JOY: no `_`; named omitted axes stay unset (caller fills 0).
 */
static bool parse_axis_args(const char *s, int kind, AxisParsed *out) {
  axis_parsed_clear(out);
  skip_ws(&s);
  if (!*s) {
    return false;
  }
  bool named = axis_letter(*s) >= 0;
  out->named = named;
  if (named) {
    while (*s) {
      skip_ws(&s);
      if (!*s) {
        break;
      }
      int ax = axis_letter(*s);
      if (ax < 0) {
        return false;
      }
      ++s;
      if (out->set[ax] || !axis_fitted(ax)) {
        return false;
      }
      skip_ws(&s);
      if (kind == AXIS_WINDOW && take_none(&s)) {
        out->set[ax] = true;
        out->v[ax] = NAN;
        continue;
      }
      if (kind == AXIS_PATH) {
        int iv = 0;
        if (!take_int(&s, &iv) || iv < -32768 || iv > 32767) {
          return false;
        }
        out->set[ax] = true;
        out->v[ax] = (float)iv;
        continue;
      }
      float fv;
      if (!take_float(&s, &fv)) {
        return false;
      }
      out->set[ax] = true;
      out->v[ax] = fv;
    }
    return any_set(out);
  }

  int slot = 0;
  while (*s && slot < MC_CH_MAX) {
    skip_ws(&s);
    if (!*s) {
      break;
    }
    if (axis_letter(*s) >= 0) {
      return false; /* mixed */
    }
    if (!axis_fitted(slot)) {
      return false;
    }
    if (*s == '_') {
      if (kind == AXIS_JOY) {
        return false;
      }
      ++s;
      if (kind == AXIS_PATH) {
        out->set[slot] = true;
        out->v[slot] = 0.0f;
      } else if (kind == AXIS_MOVE) {
        out->set[slot] = true;
        out->v[slot] = NAN;
      } else {
        out->set[slot] = false;
        out->v[slot] = NAN;
      }
      ++slot;
      continue;
    }
    if (kind == AXIS_WINDOW && take_none(&s)) {
      out->set[slot] = true;
      out->v[slot] = NAN;
      ++slot;
      continue;
    }
    if (kind == AXIS_PATH) {
      int iv = 0;
      if (!take_int(&s, &iv) || iv < -32768 || iv > 32767) {
        return false;
      }
      out->set[slot] = true;
      out->v[slot] = (float)iv;
      ++slot;
      continue;
    }
    float fv;
    if (!take_float(&s, &fv)) {
      return false;
    }
    out->set[slot] = true;
    out->v[slot] = fv;
    ++slot;
  }
  skip_ws(&s);
  return !*s && slot >= 1;
}

static void parsed_to_dest(const AxisParsed *p, float dest[MC_CH_MAX]) {
  for (int i = 0; i < MC_CH_MAX; ++i) {
    dest[i] = p->set[i] ? p->v[i] : NAN;
  }
}

static bool parse_two_ints(const char *s, int *a, int *b) {
  if (!s || !*s || !a || !b) {
    return false;
  }
  char *end = nullptr;
  long v1 = strtol(s, &end, 10);
  if (end == s) {
    return false;
  }
  while (*end == ' ' || *end == '\t') {
    ++end;
  }
  char *end2 = nullptr;
  long v2 = strtol(end, &end2, 10);
  if (end2 == end) {
    return false;
  }
  while (*end2 == ' ' || *end2 == '\t') {
    ++end2;
  }
  if (*end2 != 0) {
    return false;
  }
  *a = (int)v1;
  *b = (int)v2;
  return true;
}

/** Match ED0..3 / EI0..3; return 0..3 or -1. rest is remaining args. */
static int match_ext_index0_cmd(const char *cmd, const char *verb, const char **rest_out) {
  const char *rest = cmd;
  if (!cmd_verb(cmd, verb, &rest)) {
    return -1;
  }
  skip_ws(&rest);
  if (*rest < '0' || *rest > '3') {
    return -2;
  }
  int ch = *rest - '0';
  ++rest;
  skip_ws(&rest);
  *rest_out = rest;
  return ch;
}

static bool parse_int_arg(const char *s, int *out) {
  if (!s || !*s) {
    return false;
  }
  char *end = nullptr;
  long v = strtol(s, &end, 10);
  if (end == s) {
    return false;
  }
  while (*end == ' ' || *end == '\t') {
    ++end;
  }
  if (*end != 0) {
    return false;
  }
  *out = (int)v;
  return true;
}

/** Bool arg: bare → toggle current; otherwise require 0|1. */
static bool parse_bool_arg_or_toggle(const char *rest, bool current, bool *out) {
  if (!rest || !*rest) {
    *out = !current;
    return true;
  }
  int v = 0;
  if (!parse_int_arg(rest, &v) || (v != 0 && v != 1)) {
    return false;
  }
  *out = (v != 0);
  return true;
}

static void reply_query(const char *tag, const char *value) {
  char buf[96];
  snprintf(buf, sizeof(buf), "%s:%s\n", tag, value);
  protocol_write(buf);
}

static void reply_query_int(const char *tag, int v) {
  char buf[32];
  snprintf(buf, sizeof(buf), "%s:%d\n", tag, v);
  protocol_write(buf);
}

static void reply_query_float(const char *tag, float v) {
  char buf[40];
  snprintf(buf, sizeof(buf), "%s:%.2f\n", tag, (double)v);
  protocol_write(buf);
}

static void fmt_window_field(char *out, size_t n, float v) {
  if (isnan(v)) {
    snprintf(out, n, "-");
  } else {
    snprintf(out, n, "%.2f", (double)v);
  }
}

static void reply_pipe_floats(const char *tag, const float *v, int n, bool window) {
  char buf[192];
  size_t used = 0;
  int wr = snprintf(buf, sizeof(buf), "%s:", tag);
  if (wr < 0) {
    return;
  }
  used = (size_t)wr;
  for (int i = 0; i < n && used + 24 < sizeof(buf); ++i) {
    if (i > 0) {
      buf[used++] = ' ';
      buf[used++] = '|';
      buf[used++] = ' ';
      buf[used] = 0;
    }
    char f[16];
    if (window) {
      fmt_window_field(f, sizeof(f), v[i]);
    } else {
      snprintf(f, sizeof(f), "%.2f", (double)v[i]);
    }
    size_t fl = strlen(f);
    if (used + fl + 1 >= sizeof(buf)) {
      break;
    }
    memcpy(buf + used, f, fl + 1);
    used += fl;
  }
  if (used + 1 < sizeof(buf)) {
    buf[used++] = '\n';
    buf[used] = 0;
  }
  protocol_write(buf);
}

static void reply_window(const char *tag) {
  float v[MC_CH_MAX];
  int n = config_axis_count();
  bool left = (tag[1] == 'L' || tag[1] == 'l');
  for (int i = 0; i < n; ++i) {
    v[i] = left ? session_effective_left(i) : session_effective_right(i);
  }
  reply_pipe_floats(tag, v, n, true);
}

static void cg_dump_one(const char *key, const char *value, void *ctx) {
  (void)ctx;
  char buf[96];
  snprintf(buf, sizeof(buf), "CG:%s=%s\n", key, value);
  protocol_write(buf);
}

static const char *split_two(char *buf, char **second) {
  char *p = buf;
  while (*p == ' ' || *p == '\t') {
    ++p;
  }
  char *first = p;
  while (*p && *p != ' ' && *p != '\t') {
    ++p;
  }
  if (*p) {
    *p++ = 0;
    while (*p == ' ' || *p == '\t') {
      ++p;
    }
  }
  *second = p;
  return first;
}

static bool begin_wait_cmd(ProtocolWaitKind kind, const char *rest) {
  float timeout_s = -1.0f;
  if (*rest) {
    if (!parse_float_arg(rest, &timeout_s) || timeout_s < 0.0f) {
      protocol_error("parse", "W timeout");
      return false;
    }
  }
  protocol_begin_wait(kind, timeout_s);
  return true;
}

static bool wait_cruise_done(void) {
  McStatus st;
  motion_get_status(&st);
  if (!st.moving) {
    return true;
  }
  return protocol_state_letter() == 'M';
}

static bool wait_not_cruise_done(void) { return protocol_state_letter() != 'M'; }

static bool begin_wait_if_needed(ProtocolWaitKind kind, const char *rest, bool already_done) {
  float timeout_s = -1.0f;
  if (*rest) {
    if (!parse_float_arg(rest, &timeout_s) || timeout_s < 0.0f) {
      protocol_error("parse", "W timeout");
      return false;
    }
  }
  if (already_done) {
    return false;
  }
  protocol_begin_wait(kind, timeout_s);
  return true;
}

/** Parse required pos and optional timeout_s (default -1 = none). */
static bool parse_pos_timeout(const char *s, float *pos, float *timeout_s) {
  *timeout_s = -1.0f;
  static char buf[CFG_LINE_MAX];
  strncpy(buf, s ? s : "", sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = 0;
  char *second = nullptr;
  const char *first = split_two(buf, &second);
  if (!first || !*first) {
    return false;
  }
  if (!parse_float_arg(first, pos)) {
    return false;
  }
  if (second && *second) {
    if (!parse_float_arg(second, timeout_s) || *timeout_s < 0.0f) {
      return false;
    }
  }
  return true;
}

static bool begin_wait_pos_cmd(const char *rest) {
  float pos = 0.0f;
  float timeout_s = -1.0f;
  if (!parse_pos_timeout(rest, &pos, &timeout_s)) {
    protocol_error("parse", "WP args");
    return false;
  }
  McStatus st;
  motion_get_status(&st);
  if (!st.moving && !st.homing) {
    return false;
  }
  int ch = motion_master_channel();
  if (ch < 0 || ch >= MC_CH_MAX) {
    return false;
  }
  float vel = st.vel[ch];
  int sign;
  if (st.moving && vel > 0.01f) {
    sign = 1;
  } else if (st.moving && vel < -0.01f) {
    sign = -1;
  } else {
    return false; /* dir-pause / ~0 vel: treat as idle */
  }
  if ((sign > 0 && st.pos[ch] >= pos) || (sign < 0 && st.pos[ch] <= pos)) {
    return false;
  }
  protocol_begin_wait_pos(pos, sign, timeout_s);
  return true;
}

typedef struct {
  const char *sh;
  const char *desc;
} HelpRow;

/* Two columns. Group order: S → G → I → M → P → W → E → Special → C → V → now. */
static const HelpRow k_help_rows[] = {
    {"SS", "Set Speed, cruise mm/s"},
    {"SA", "Set Accel, mm/s2"},
    {"SE", "Set Enable, 0|1; bare toggles"},
    {"ST", "Set Terminal, 0|1; bare toggles"},
    {"SV", "Set Verbose, 0|1; bare toggles"},
    {"SD", "Set Debug, USB level 0..5"},
    {"SL", "Set Left, session window min"},
    {"SR", "Set Right, session window max"},
    {"SP", "Set Position, bare/0 = here is 0"},
    {"GS", "Get Speed, session cruise mm/s"},
    {"GA", "Get Accel, session mm/s2"},
    {"GE", "Get Enable, driver state 0|1"},
    {"GT", "Get Terminal, 0|1"},
    {"GV", "Get Verbose, 0|1"},
    {"GD", "Get Debug, USB level"},
    {"GL", "Get Left, session window min"},
    {"GR", "Get Right, session window max"},
    {"IM", "Is Moving, 0|1"},
    {"IH", "Is Homing, 0|1"},
    {"IL", "Is Limit, at soft-limit pose?"},
    {"IE", "Is Error, DRV_ERROR 0|1"},
    {"IP", "Is Position, packed | groups"},
    {"IA", "Is Axis, motors+servos"},
    {"IT", "Is Target, mm or -"},
    {"IR", "Is Ready, motion 0|1"},
    {"IW", "Is Waiting, 0|1"},
    {"ID", "Is Diag, motion counters"},
    {"IC", "Is Cause, last reset reason"},
    {"IG", "Is GPIO, pin name / desc table"},
    {"MT", "Move To, absolute mm"},
    {"MB", "Move By, relative mm"},
    {"MJ", "Move Joy, % of SS signed"},
    {"MH", "Move Home, cycle 1|2|3"},
    {"MS", "Move Stop, soft decelerate"},
    {"ME", "Move E-Stop, Halt; EN off"},
    {"PC", "Path Clear, empty buffer"},
    {"PD", "Path Data, um -32768..32767"},
    {"PG", "Path Go, [start end] 0-based; reverse if start>end"},
    {"PN", "Path Number, sample count"},
    {"PI", "Path Index, playhead 0-based"},
    {"PS", "Path Slice, us >=1000; bare resets"},
    {"WT", "Wait Time, delay sec (default 1)"},
    {"WM", "Wait Moving, until move done"},
    {"WH", "Wait Homing, until home done"},
    {"WP", "Wait Pos, until master pos"},
    {"WC", "Wait Cruise, until cruise or idle"},
    {"WN", "Wait Not cruise, until not M"},
    {"EO", "Ext Out, EO1..4 0|1; bare toggles"},
    {"ED", "Ext Dir, ED0..3 I|O|T (in/out/OC)"},
    {"EI", "Ext In, EI0..3 read pin 0|1"},
    {"RB", "Reboot, soft MCU reset"},
    {"BE", "Beep, pulse buzzer [ms] 1..1000"},
    {"CT", "Camera Trigger, pulse [ms] 1..60000"},
    {"HL/?", "Help, list commands"},
    {"CS", "Config Set, persistent key"},
    {"CR", "Config Reset, all defaults"},
    {"CG", "Config Get, key(s)"},
    {"VA", "Version About, about string"},
    {"VH", "Version Hello, welcome banner"},
    {"VF", "Version FW, firmware version"},
    {"VP", "Version Protocol, protocol version"},
    {"VG", "Version GPIO, PIN_*= lines"},
    {"#", "Status now, compact line"},
    {"!", "Soft stop now"},
    {"ESC", "Halt now (same as ME)"},
};

static void cmd_help(void) {
  char line[88];
  protocol_writeln("Cmd  Description");
  protocol_writeln("---  -----------");
  for (size_t i = 0; i < sizeof(k_help_rows) / sizeof(k_help_rows[0]); ++i) {
    const HelpRow *r = &k_help_rows[i];
    snprintf(line, sizeof(line), "%-5s %s", r->sh, r->desc);
    protocol_writeln(line);
  }
  protocol_writeln("/ comments to end of line");
  protocol_writeln("Now: # status  ! soft-stop  ESC halt");
  protocol_writeln("? is help (needs newline), not status");
}

typedef struct {
  int gp;
  const char *name;
  const char *desc;
} PinIndexRow;

static int pin_index_cmp(const void *a, const void *b) {
  const PinIndexRow *ra = (const PinIndexRow *)a;
  const PinIndexRow *rb = (const PinIndexRow *)b;
  return ra->gp - rb->gp;
}

static void cmd_pinout_index(void) {
  PinIndexRow rows[48];
  size_t n = 0;

#define PIN_IX_ADD(gpio, nam, dsc)                                             \
  do {                                                                         \
    if (n < sizeof(rows) / sizeof(rows[0])) {                                  \
      rows[n].gp = (gpio);                                                     \
      rows[n].name = (nam);                                                    \
      rows[n].desc = (dsc);                                                    \
      ++n;                                                                     \
    }                                                                          \
  } while (0)

  PIN_IX_ADD(PIN_EXT_1, "EXT_1", "Extender output 1 (EO1)");
  PIN_IX_ADD(PIN_EXT_2, "EXT_2", "Extender output 2 (EO2)");
  PIN_IX_ADD(PIN_EXT_3, "EXT_3", "Extender output 3 (EO3)");
  if (config_ext_available(3)) {
    PIN_IX_ADD(PIN_EXT_4, "EXT_4", "Extender output 4 (EO4)");
  }
  if (config_servo_count() >= 1) {
    PIN_IX_ADD(PIN_SERVO_1, "SERVO_1", "RC servo 1 PWM");
  }
  if (config_servo_count() >= 2) {
    PIN_IX_ADD(PIN_SERVO_2, "SERVO_2", "RC servo 2 PWM");
  }
  if (config_servo_count() >= 3) {
    PIN_IX_ADD(PIN_SERVO_3, "SERVO_3", "RC servo 3 PWM");
  }
  PIN_IX_ADD(PIN_DRV_STEP_1, "DRV_STEP_1", "STEP to driver");
  PIN_IX_ADD(PIN_DRV_DIR_1, "DRV_DIR_1", "DIR to driver");
  PIN_IX_ADD(PIN_DRV_ENABLE, "DRV_ENABLE", "Driver enable (all axes)");
  PIN_IX_ADD(PIN_DRV_ERROR_1, "DRV_ERROR_1", "Driver fault / E-stop input");
  PIN_IX_ADD(PIN_SW_LIMIT_L_1, "SW_LIMIT_L_1", "Hard limit left");
  PIN_IX_ADD(PIN_SW_LIMIT_R_1, "SW_LIMIT_R_1", "Hard limit right");
  if (config_axis2_enabled()) {
    PIN_IX_ADD(PIN_DRV_STEP_2, "DRV_STEP_2", "STEP axis2");
    PIN_IX_ADD(PIN_DRV_DIR_2, "DRV_DIR_2", "DIR axis2");
    PIN_IX_ADD(PIN_DRV_ERROR_2, "DRV_ERROR_2", "Fault / E-stop axis2");
    PIN_IX_ADD(PIN_SW_LIMIT_L_2, "SW_LIMIT_L_2", "Hard limit left axis2");
    PIN_IX_ADD(PIN_SW_LIMIT_R_2, "SW_LIMIT_R_2", "Hard limit right axis2");
  }
  if (config_axis3_enabled()) {
    PIN_IX_ADD(PIN_DRV_STEP_3, "DRV_STEP_3", "STEP axis3");
    PIN_IX_ADD(PIN_DRV_DIR_3, "DRV_DIR_3", "DIR axis3");
    PIN_IX_ADD(PIN_DRV_ERROR_3, "DRV_ERROR_3", "Fault / E-stop axis3");
    PIN_IX_ADD(PIN_SW_LIMIT_L_3, "SW_LIMIT_L_3", "Hard limit left axis3");
    PIN_IX_ADD(PIN_SW_LIMIT_R_3, "SW_LIMIT_R_3", "Hard limit right axis3");
  }
#ifdef PIN_CAMERA_CTRL
  PIN_IX_ADD(PIN_CAMERA_CTRL, "CAMERA_CTRL", "Camera OC sink (CT) / listen, low-active");
#endif
  PIN_IX_ADD(PIN_UART_TX, "UART_TX", "UART TX to UIC (115200 baud)");
  PIN_IX_ADD(PIN_UART_RX, "UART_RX", "UART RX from UIC (115200 baud)");
  PIN_IX_ADD(PIN_LED, "LED", "Status / heartbeat LED");
  if (config_get()->buzzer_use && PIN_BUZZER != PIN_LED) {
    PIN_IX_ADD(PIN_BUZZER, "BUZZER", "Piezo pulse (BE)");
  }
#ifdef DEBUG_HW
  if (dbg_hw_allowed()) {
    PIN_IX_ADD(PIN_DBG_FIFO, "DBG_FIFO", "Scope: TX FIFO non-empty");
    PIN_IX_ADD(PIN_DBG_MOV, "DBG_MOV", "Scope: moving or homing");
    PIN_IX_ADD(PIN_DBG_MOV_CONST, "DBG_MOV_CONST", "Scope: cruise (equal delays)");
    PIN_IX_ADD(PIN_DBG_CMD, "DBG_CMD", "Scope: command handler busy");
    PIN_IX_ADD(PIN_DBG_IRQ, "DBG_IRQ", "Scope: PIO TX-not-full IRQ pulse");
    PIN_IX_ADD(PIN_DBG_UNDERRUN, "DBG_UNDERRUN", "Scope: FIFO underrun pulse");
  }
#endif
#undef PIN_IX_ADD

  qsort(rows, n, sizeof(rows[0]), pin_index_cmp);

  char line[88];
  protocol_writeln("GP   Name           Description");
  protocol_writeln("---- -------------- -----------------------------------------------");
  for (size_t i = 0; i < n; ++i) {
    char gpbuf[8];
    snprintf(gpbuf, sizeof(gpbuf), "GP%d", rows[i].gp);
    snprintf(line, sizeof(line), "%-4s %-14s %s", gpbuf, rows[i].name, rows[i].desc);
    protocol_writeln(line);
  }
}

/** Commands allowed while PIN_DRV_ERROR_* is asserted (diagnostics / config / halt). */
static bool emo_command_allowed(const char *cmd) {
  const char *rest = cmd;
  static const char *k[] = {"IE", "IA", "ID", "IC", "VA", "VF", "VP", "VG", "VH", "IG",
                            "HL", "CS", "CR", "CG", "ME", "RB", "BE", "CT", "EO", "EI"};
  for (size_t i = 0; i < sizeof(k) / sizeof(k[0]); ++i) {
    if (cmd_verb(cmd, k[i], &rest)) {
      return true;
    }
  }
  return cmd[0] == '?';
}

/** Commands allowed while path-mode (PG) is active: safety/queries/stop/PD (live-move) only. */
static bool path_command_allowed(const char *cmd) {
  const char *rest = cmd;
  static const char *k[] = {
      "MS", "ME", "RB", "PD", "PN", "PI", "IM", "IH", "IL", "IE", "IP", "IA", "IT",
      "IR", "IW", "ID", "IC", "IG", "GS", "GA", "GE", "GT", "GV", "GD", "GL",
      "GR", "VA", "VF", "VP", "VG", "VH", "HL", "CG", "BE", "CT", "EI"};
  for (size_t i = 0; i < sizeof(k) / sizeof(k[0]); ++i) {
    if (cmd_verb(cmd, k[i], &rest)) {
      return true;
    }
  }
  return cmd[0] == '?';
}

/** Match EO + channel; return 0..3 or -1. Rest is the optional 0|1. */
static int match_ext_cmd(const char *cmd, const char **rest_out) {
  const char *rest = cmd;
  if (!cmd_verb(cmd, "EO", &rest)) {
    return -1;
  }
  skip_ws(&rest);
  if (*rest < '1' || *rest > '4') {
    return -2; /* EO without 1..4 */
  }
  int ch = *rest - '1';
  ++rest;
  skip_ws(&rest);
  *rest_out = rest;
  return ch;
}

bool protocol_exec_command(const char *cmd) {
  while (*cmd == ' ' || *cmd == '\t') {
    ++cmd;
  }
  if (*cmd == 0) {
    return false;
  }

  const char *rest = cmd;
  McStatus st;
  motion_get_status(&st);
  McSession *sess = session_get();

  if (st.drv_error && !emo_command_allowed(cmd)) {
    protocol_error("emo", "active");
    return false;
  }

  if (motion_path_is_active() && !path_command_allowed(cmd)) {
    protocol_error("busy", "path active");
    return false;
  }

  /* --- EO extender outputs --- */
  {
    int xi = match_ext_cmd(cmd, &rest);
    if (xi == -2) {
      protocol_error("parse", "EO 1..4");
      return false;
    }
    if (xi >= 0) {
      bool on = false;
      if (!parse_bool_arg_or_toggle(rest, board_ext_get(xi), &on)) {
        protocol_error("parse", "EO 0|1");
        return false;
      }
      if (!board_ext_set(xi, on)) {
        protocol_error("parse", "EO");
        return false;
      }
      return false;
    }
  }

  {
    int xi = match_ext_index0_cmd(cmd, "ED", &rest);
    if (xi == -2) {
      protocol_error("parse", "ED 0..3");
      return false;
    }
    if (xi >= 0) {
      if (!rest || !*rest) {
        protocol_error("parse", "ED I|O|T");
        return false;
      }
      char m = rest[0];
      if (m >= 'a' && m <= 'z') {
        m = (char)(m - 'a' + 'A');
      }
      BoardExtMode mode = BOARD_EXT_IN;
      if (m == 'I') {
        mode = BOARD_EXT_IN;
      } else if (m == 'O') {
        mode = BOARD_EXT_OUT;
      } else if (m == 'T') {
        mode = BOARD_EXT_OC;
      } else {
        protocol_error("parse", "ED I|O|T");
        return false;
      }
      ++rest;
      skip_ws(&rest);
      if (*rest) {
        protocol_error("parse", "ED I|O|T");
        return false;
      }
      if (!board_ext_set_mode(xi, mode)) {
        protocol_error("parse", "ED");
        return false;
      }
      return false;
    }
  }

  {
    int xi = match_ext_index0_cmd(cmd, "EI", &rest);
    if (xi == -2) {
      protocol_error("parse", "EI 0..3");
      return false;
    }
    if (xi >= 0) {
      if (rest && *rest) {
        protocol_error("parse", "EI 0..3");
        return false;
      }
      int high = 0;
      if (!board_ext_read_pin(xi, &high)) {
        protocol_error("parse", "EI");
        return false;
      }
      char line[24];
      snprintf(line, sizeof(line), "EI:%d %d\n", xi, high);
      protocol_write(line);
      return false;
    }
  }

  if (match_any(cmd, &rest, "BE", nullptr, nullptr)) {
    int ms = 100;
    if (!parse_optional_pulse_ms(rest, 1, 1000, &ms)) {
      protocol_error("parse", "BE ms");
      return false;
    }
    board_buzzer_pulse((unsigned)ms);
    return false;
  }

  if (match_any(cmd, &rest, "CT", nullptr, nullptr)) {
    int ms = 100;
    if (!parse_optional_pulse_ms(rest, 1, 60000, &ms)) {
      protocol_error("parse", "CT ms");
      return false;
    }
    board_camera_ctrl_pulse((unsigned)ms);
    if (session_get()->verbose) {
      protocol_verbose_reset_dedupe();
      protocol_send_status();
    }
    return false;
  }

  /* --- M motion (longer prefixes before M) --- */
  if (match_any(cmd, &rest, "MT", "MoveTo", nullptr)) {
    AxisParsed p;
    if (!parse_axis_args(rest, AXIS_MOVE, &p)) {
      protocol_error("parse", "MT args");
      return false;
    }
    if (!st.enabled) {
      protocol_error("disabled", "enable first");
      return false;
    }
    float dest[MC_CH_MAX];
    parsed_to_dest(&p, dest);
    McStatus cur;
    motion_get_status(&cur);
    int n = config_axis_count();
    for (int i = 0; i < n; ++i) {
      if (!isnan(dest[i]) && fabsf(dest[i] - cur.pos[i]) < 1e-4f) {
        dest[i] = NAN;
      }
    }
    bool ok = motion_move_to_n(dest);
    if (!ok) {
      motion_get_status(&st);
      const char *code = "soft";
      if (st.drv_error) {
        code = "emo";
      } else if (st.hard_limit) {
        code = "hard";
      }
      protocol_error(code, "move rejected");
      return false;
    }
    return false;
  }

  if (match_any(cmd, &rest, "MJ", nullptr, nullptr)) {
    AxisParsed jp;
    if (!parse_axis_args(rest, AXIS_JOY, &jp) || (!jp.named && !jp.set[0])) {
      protocol_error("parse", "MJ args");
      return false;
    }
    float pct[MC_CH_MAX];
    int n = config_axis_count();
    for (int i = 0; i < MC_CH_MAX; ++i) {
      if (i < n) {
        pct[i] = jp.set[i] ? jp.v[i] : (jp.named ? 0.0f : (i == 0 ? 0.0f : 0.0f));
        if (!jp.named && !jp.set[i]) {
          pct[i] = 0.0f;
        }
      } else {
        pct[i] = NAN;
      }
    }
    if (!st.enabled) {
      protocol_error("disabled", "enable first");
      return false;
    }
    if (!motion_joy(pct)) {
      protocol_error("disabled", "enable first");
    }
    return false;
  }

  if (match_any(cmd, &rest, "MH", "MoveHome", nullptr)) {
    if (!st.enabled) {
      protocol_error("disabled", "enable first");
      return false;
    }
    int axis = 1;
    if (*rest) {
      if (!parse_int_arg(rest, &axis) || axis < 1 || axis > 3 ||
          axis > config_motor_count()) {
        protocol_error("parse", "MH 1|2|3");
        return false;
      }
    }
    if (!motion_home(axis)) {
      protocol_error("home", "cfg");
    }
    return false;
  }

  if (match_any(cmd, &rest, "MS", "MoveStop", "Stop")) {
    if (motion_path_is_active()) {
      motion_path_abort_to_planner();
    }
    motion_stop();
    return false;
  }

  if (match_any(cmd, &rest, "MB", nullptr, nullptr)) {
    AxisParsed p;
    if (!parse_axis_args(rest, AXIS_MOVE, &p)) {
      protocol_error("parse", "MB args");
      return false;
    }
    if (!st.enabled) {
      protocol_error("disabled", "enable first");
      return false;
    }
    McStatus cur;
    motion_get_status(&cur);
    float dest[MC_CH_MAX];
    int n = config_axis_count();
    for (int i = 0; i < MC_CH_MAX; ++i) {
      dest[i] = NAN;
    }
    for (int i = 0; i < n; ++i) {
      if (!p.set[i] || isnan(p.v[i])) {
        continue;
      }
      if (fabsf(p.v[i]) < 1e-6f) {
        continue;
      }
      dest[i] = cur.pos[i] + p.v[i];
    }
    bool ok = motion_move_to_n(dest);
    if (!ok) {
      motion_get_status(&st);
      const char *code = "soft";
      if (st.drv_error) {
        code = "emo";
      } else if (st.hard_limit) {
        code = "hard";
      }
      protocol_error(code, "move rejected");
      return false;
    }
    return false;
  }

  /* --- P path (host-authored motion path) --- */
  if (match_any(cmd, &rest, "PC", "PathClear", nullptr)) {
    if (!motion_path_clear()) {
      protocol_error("busy", "path active");
    }
    return false;
  }

  if (match_any(cmd, &rest, "PD", nullptr, nullptr)) {
    AxisParsed pp;
    if (!parse_axis_args(rest, AXIS_PATH, &pp)) {
      protocol_error("parse", "PD -32768..32767");
      return false;
    }
    int16_t samp[MC_CH_MAX];
    for (int i = 0; i < MC_CH_MAX; ++i) {
      samp[i] = pp.set[i] ? (int16_t)pp.v[i] : 0;
    }
    if (!motion_path_addn(samp)) {
      protocol_error("full", "path buffer full");
    }
    return false;
  }

  if (match_any(cmd, &rest, "PG", "PathGo", nullptr)) {
    if (!st.enabled) {
      protocol_error("disabled", "enable first");
      return false;
    }
    if (motion_path_count() == 0) {
      protocol_error("empty", "path buffer empty");
      return false;
    }
    if (!*rest) {
      if (!motion_path_go()) {
        protocol_error("busy", "path active");
      }
      return false;
    }
    int start_i = 0;
    int end_i = 0;
    if (!parse_two_ints(rest, &start_i, &end_i)) {
      protocol_error("parse", "PG start end");
      return false;
    }
    uint32_t n = motion_path_count();
    if (start_i < 0 || end_i < 0 || (uint32_t)start_i >= n || (uint32_t)end_i >= n) {
      protocol_error("range", "PG start end");
      return false;
    }
    if (!motion_path_go_range((uint32_t)start_i, (uint32_t)end_i)) {
      protocol_error("busy", "path active");
    }
    return false;
  }

  if (match_any(cmd, &rest, "PN", "PathNumber", nullptr)) {
    reply_query_int("PN", (int)motion_path_count());
    return false;
  }

  if (match_any(cmd, &rest, "PI", "PathIndex", nullptr)) {
    reply_query_int("PI", (int)motion_path_play_index());
    return false;
  }

  if (match_any(cmd, &rest, "PS", "PathSlice", nullptr)) {
    if (!*rest) {
      session_reset_path_slice();
      return false;
    }
    int us;
    if (!parse_int_arg(rest, &us) || us < PATH_SLICE_US_MIN) {
      protocol_error("parse", "PS >=1000");
      return false;
    }
    if (!motion_path_set_slice_us((uint32_t)us)) {
      protocol_error("busy", "path active");
    }
    return false;
  }

  /* --- S session set --- */
  if (match_any(cmd, &rest, "SS", "SetSpeed", nullptr)) {
    if (!*rest) {
      session_reset_speed();
      motion_set_speed(sess->speed_mm_s);
      return false;
    }
    float v;
    if (!parse_float_arg(rest, &v) || v < 0.0f) {
      protocol_error("parse", "SS args");
      return false;
    }
    if (v > config_get()->max_speed_mm_s) {
      protocol_error("limit", "SS > max_speed");
      return false;
    }
    motion_set_speed(v);
    return false;
  }

  if (match_any(cmd, &rest, "SA", "SetAccel", nullptr)) {
    if (!*rest) {
      session_reset_accel();
      motion_set_accel(sess->accel_mm_s2);
      return false;
    }
    float a;
    if (!parse_float_arg(rest, &a) || a <= 0.0f) {
      protocol_error("parse", "SA args");
      return false;
    }
    if (a > config_get()->max_accel_mm_s2) {
      protocol_error("limit", "SA > max_accel");
      return false;
    }
    motion_set_accel(a);
    return false;
  }

  if (match_any(cmd, &rest, "SE", "SetEnable", "Enable")) {
    motion_get_status(&st);
    bool on = false;
    if (!parse_bool_arg_or_toggle(rest, st.enabled, &on)) {
      protocol_error("parse", "SE args");
      return false;
    }
    motion_enable(on);
    return false;
  }

  if (match_any(cmd, &rest, "ST", "SetTerminal", nullptr)) {
    bool on = false;
    if (!parse_bool_arg_or_toggle(rest, sess->terminal != 0, &on)) {
      protocol_error("parse", "ST args");
      return false;
    }
    sess->terminal = on ? 1 : 0;
    return false;
  }

  if (match_any(cmd, &rest, "SV", "SetVerbose", nullptr)) {
    bool on = false;
    if (!parse_bool_arg_or_toggle(rest, sess->verbose != 0, &on)) {
      protocol_error("parse", "SV args");
      return false;
    }
    sess->verbose = on ? 1 : 0;
    return false;
  }

  if (match_any(cmd, &rest, "SD", "SetDebug", nullptr)) {
    if (!*rest) {
      config_get()->init_debug_level = CFG_DEFAULT_INIT_DEBUG_LEVEL;
      return false;
    }
    int level = 0;
    if (!parse_int_arg(rest, &level) || level < 0 || level > 5) {
      protocol_error("parse", "SD 0..5");
      return false;
    }
    config_get()->init_debug_level = level;
    return false;
  }

  if (match_any(cmd, &rest, "SL", "SetLeft", nullptr)) {
    if (!*rest) {
      motion_reset_window_left();
      return false;
    }
    AxisParsed p;
    if (!parse_axis_args(rest, AXIS_WINDOW, &p)) {
      protocol_error("parse", "SL args");
      return false;
    }
    if (!motion_set_window_left(p.set, p.v)) {
      protocol_error("limit", "SL");
    }
    return false;
  }

  if (match_any(cmd, &rest, "SR", "SetRight", nullptr)) {
    if (!*rest) {
      motion_reset_window_right();
      return false;
    }
    AxisParsed p;
    if (!parse_axis_args(rest, AXIS_WINDOW, &p)) {
      protocol_error("parse", "SR args");
      return false;
    }
    if (!motion_set_window_right(p.set, p.v)) {
      protocol_error("limit", "SR");
    }
    return false;
  }

  if (match_any(cmd, &rest, "SP", "SetPosition", nullptr)) {
    if (motion_is_busy() || motion_path_is_active()) {
      protocol_error("busy", "SP");
      return false;
    }
    float dest[MC_CH_MAX];
    int n = config_axis_count();
    if (!*rest) {
      for (int i = 0; i < MC_CH_MAX; ++i) {
        dest[i] = (i < n) ? 0.0f : NAN;
      }
    } else {
      AxisParsed p;
      if (!parse_axis_args(rest, AXIS_MOVE, &p)) {
        protocol_error("parse", "SP args");
        return false;
      }
      parsed_to_dest(&p, dest);
    }
    if (!motion_set_position(dest)) {
      protocol_error("busy", "SP");
    }
    return false;
  }

  /* --- G session get --- */
  if (match_any(cmd, &rest, "GS", "GetSpeed", nullptr)) {
    reply_query_float("GS", sess->speed_mm_s);
    return false;
  }
  if (match_any(cmd, &rest, "GA", "GetAccel", nullptr)) {
    reply_query_float("GA", sess->accel_mm_s2);
    return false;
  }
  if (match_any(cmd, &rest, "GE", "GetEnable", nullptr)) {
    motion_get_status(&st);
    reply_query_int("GE", st.enabled ? 1 : 0);
    return false;
  }
  if (match_any(cmd, &rest, "GT", "GetTerminal", nullptr)) {
    reply_query_int("GT", sess->terminal ? 1 : 0);
    return false;
  }
  if (match_any(cmd, &rest, "GV", "GetVerbose", nullptr)) {
    reply_query_int("GV", sess->verbose ? 1 : 0);
    return false;
  }
  if (match_any(cmd, &rest, "GD", "GetDebug", nullptr)) {
    reply_query_int("GD", config_get()->init_debug_level);
    return false;
  }
  if (match_any(cmd, &rest, "GL", "GetLeft", nullptr)) {
    reply_window("GL");
    return false;
  }
  if (match_any(cmd, &rest, "GR", "GetRight", nullptr)) {
    reply_window("GR");
    return false;
  }

  /* --- I status --- */
  if (match_any(cmd, &rest, "IM", "IsMoving", nullptr)) {
    motion_get_status(&st);
    reply_query_int("IM", st.moving ? 1 : 0);
    return false;
  }
  if (match_any(cmd, &rest, "IH", "IsHoming", nullptr)) {
    motion_get_status(&st);
    reply_query_int("IH", st.homing ? 1 : 0);
    return false;
  }
  if (match_any(cmd, &rest, "IL", "IsLimit", nullptr)) {
    motion_get_status(&st);
    reply_query_int("IL", st.at_soft_limit ? 1 : 0);
    return false;
  }
  if (match_any(cmd, &rest, "IE", "IsError", nullptr)) {
    motion_get_status(&st);
    reply_query_int("IE", st.drv_error ? 1 : 0);
    return false;
  }
  if (match_any(cmd, &rest, "IP", "IsPosition", nullptr)) {
    motion_get_status(&st);
    int n = config_axis_count();
    reply_pipe_floats("IP", st.pos, n, false);
    return false;
  }
  if (match_any(cmd, &rest, "IA", "Axis", "IsAxis")) {
    reply_query_int("IA", config_axis_count());
    return false;
  }
  if (match_any(cmd, &rest, "IT", "IsTarget", nullptr)) {
    motion_get_status(&st);
    if (st.has_target) {
      reply_query_float("IT", st.target[0]);
    } else {
      reply_query("IT", "-");
    }
    return false;
  }
  if (match_any(cmd, &rest, "IR", "IsReady", nullptr)) {
    motion_get_status(&st);
    int ready = (!st.moving && !st.homing && st.enabled && !protocol_is_waiting()) ? 1 : 0;
    reply_query_int("IR", ready);
    return false;
  }
  if (match_any(cmd, &rest, "IW", "IsWaiting", nullptr)) {
    reply_query_int("IW", protocol_is_waiting() ? 1 : 0);
    return false;
  }
  if (match_any(cmd, &rest, "ID", "IsDiag", nullptr)) {
    MotionDiag d;
    motion_diag_get(&d);
    char buf[128];
    unsigned f0 = d.fifo_min_axis[0];
    unsigned f1 = d.fifo_min_axis[1];
    unsigned f2 = d.fifo_min_axis[2];
    if (f0 == 0xFFFFFFFFu) {
      f0 = 0;
    }
    if (f1 == 0xFFFFFFFFu) {
      f1 = 0;
    }
    if (f2 == 0xFFFFFFFFu) {
      f2 = 0;
    }
    snprintf(buf, sizeof(buf),
             "underrun=%lu,%lu,%lu peak_hz=%.0f overshoot=%ld fifo_min=%u,%u,%u",
             (unsigned long)d.underrun_axis[0], (unsigned long)d.underrun_axis[1],
             (unsigned long)d.underrun_axis[2], (double)d.peak_step_hz,
             (long)d.overshoot_steps, f0, f1, f2);
    reply_query("ID", buf);
    return false;
  }
  if (match_any(cmd, &rest, "IC", nullptr, nullptr)) {
    const char *reason = "unknown";
#ifndef HOST_TEST
    switch (rp2040.getResetReason()) {
    case RP2040::PWRON_RESET:
      reason = "power";
      break;
    case RP2040::WDT_RESET:
      reason = "wdt";
      break;
    case RP2040::RUN_PIN_RESET:
      reason = "run";
      break;
    case RP2040::SOFT_RESET:
      reason = "soft";
      break;
    case RP2040::DEBUG_RESET:
      reason = "debug";
      break;
    case RP2040::GLITCH_RESET:
      reason = "glitch";
      break;
    case RP2040::BROWNOUT_RESET:
      reason = "brownout";
      break;
    default:
      reason = "unknown";
      break;
    }
#endif
    reply_query("IC", reason);
    return false;
  }

  /* --- W wait (WN/WC/WP/WM/WH before WT) --- */
  if (match_any(cmd, &rest, "WN", nullptr, nullptr)) {
    return begin_wait_if_needed(PROTOCOL_WAIT_NOT_CRUISE, rest, wait_not_cruise_done());
  }
  if (match_any(cmd, &rest, "WC", "WaitCruise", nullptr)) {
    return begin_wait_if_needed(PROTOCOL_WAIT_CRUISE, rest, wait_cruise_done());
  }
  if (match_any(cmd, &rest, "WP", "WaitPos", nullptr)) {
    return begin_wait_pos_cmd(rest);
  }
  if (match_any(cmd, &rest, "WM", "WaitMoving", nullptr)) {
    return begin_wait_cmd(PROTOCOL_WAIT_MOVING, rest);
  }
  if (match_any(cmd, &rest, "WH", "WaitHoming", nullptr)) {
    return begin_wait_cmd(PROTOCOL_WAIT_HOMING, rest);
  }
  if (match_any(cmd, &rest, "WT", nullptr, nullptr)) {
    float sec = 1.0f;
    if (*rest) {
      if (!parse_float_arg(rest, &sec) || sec < 0.0f) {
        protocol_error("parse", "WT args");
        return false;
      }
    }
    protocol_begin_wait(PROTOCOL_WAIT_DELAY, sec);
    return true;
  }

  /* --- V version --- */
  if (match_any(cmd, &rest, "VH", "VersionHello", nullptr)) {
    protocol_send_banner();
    return false;
  }
  if (match_any(cmd, &rest, "VA", "VersionAbout", nullptr)) {
    reply_query("VA", MC_VERSION_ABOUT);
    return false;
  }
  if (match_any(cmd, &rest, "VF", "VersionFW", nullptr)) {
    reply_query("VF", MC_VERSION_FW);
    return false;
  }
  if (match_any(cmd, &rest, "VP", "VersionProtocol", nullptr)) {
    reply_query("VP", MC_VERSION_PROTOCOL);
    return false;
  }
  if (match_any(cmd, &rest, "VG", "VersionGPIO", nullptr)) {
    char line[48];
    snprintf(line, sizeof(line), "VG:PIN_DRV_STEP_1=%d", PIN_DRV_STEP_1);
    protocol_writeln(line);
    snprintf(line, sizeof(line), "VG:PIN_DRV_DIR_1=%d", PIN_DRV_DIR_1);
    protocol_writeln(line);
    snprintf(line, sizeof(line), "VG:PIN_DRV_ENABLE=%d", PIN_DRV_ENABLE);
    protocol_writeln(line);
    snprintf(line, sizeof(line), "VG:PIN_DRV_ERROR_1=%d", PIN_DRV_ERROR_1);
    protocol_writeln(line);
    snprintf(line, sizeof(line), "VG:PIN_SW_LIMIT_L_1=%d", PIN_SW_LIMIT_L_1);
    protocol_writeln(line);
    snprintf(line, sizeof(line), "VG:PIN_SW_LIMIT_R_1=%d", PIN_SW_LIMIT_R_1);
    protocol_writeln(line);
    if (config_axis2_enabled()) {
      snprintf(line, sizeof(line), "VG:PIN_DRV_STEP_2=%d", PIN_DRV_STEP_2);
      protocol_writeln(line);
      snprintf(line, sizeof(line), "VG:PIN_DRV_DIR_2=%d", PIN_DRV_DIR_2);
      protocol_writeln(line);
      snprintf(line, sizeof(line), "VG:PIN_DRV_ERROR_2=%d", PIN_DRV_ERROR_2);
      protocol_writeln(line);
      snprintf(line, sizeof(line), "VG:PIN_SW_LIMIT_L_2=%d", PIN_SW_LIMIT_L_2);
      protocol_writeln(line);
      snprintf(line, sizeof(line), "VG:PIN_SW_LIMIT_R_2=%d", PIN_SW_LIMIT_R_2);
      protocol_writeln(line);
    }
    if (config_axis3_enabled()) {
      snprintf(line, sizeof(line), "VG:PIN_DRV_STEP_3=%d", PIN_DRV_STEP_3);
      protocol_writeln(line);
      snprintf(line, sizeof(line), "VG:PIN_DRV_DIR_3=%d", PIN_DRV_DIR_3);
      protocol_writeln(line);
      snprintf(line, sizeof(line), "VG:PIN_DRV_ERROR_3=%d", PIN_DRV_ERROR_3);
      protocol_writeln(line);
      snprintf(line, sizeof(line), "VG:PIN_SW_LIMIT_L_3=%d", PIN_SW_LIMIT_L_3);
      protocol_writeln(line);
      snprintf(line, sizeof(line), "VG:PIN_SW_LIMIT_R_3=%d", PIN_SW_LIMIT_R_3);
      protocol_writeln(line);
    }
#ifdef PIN_CAMERA_CTRL
    snprintf(line, sizeof(line), "VG:PIN_CAMERA_CTRL=%d", PIN_CAMERA_CTRL);
    protocol_writeln(line);
#endif
    snprintf(line, sizeof(line), "VG:PIN_EXT_1=%d", PIN_EXT_1);
    protocol_writeln(line);
    snprintf(line, sizeof(line), "VG:PIN_EXT_2=%d", PIN_EXT_2);
    protocol_writeln(line);
    snprintf(line, sizeof(line), "VG:PIN_EXT_3=%d", PIN_EXT_3);
    protocol_writeln(line);
    if (config_ext_available(3)) {
      snprintf(line, sizeof(line), "VG:PIN_EXT_4=%d", PIN_EXT_4);
      protocol_writeln(line);
    }
    if (config_servo_count() >= 1) {
      snprintf(line, sizeof(line), "VG:PIN_SERVO_1=%d", PIN_SERVO_1);
      protocol_writeln(line);
    }
    if (config_servo_count() >= 2) {
      snprintf(line, sizeof(line), "VG:PIN_SERVO_2=%d", PIN_SERVO_2);
      protocol_writeln(line);
    }
    if (config_servo_count() >= 3) {
      snprintf(line, sizeof(line), "VG:PIN_SERVO_3=%d", PIN_SERVO_3);
      protocol_writeln(line);
    }
    if (config_get()->buzzer_use && PIN_BUZZER != PIN_LED) {
      snprintf(line, sizeof(line), "VG:PIN_BUZZER=%d", PIN_BUZZER);
      protocol_writeln(line);
    }
    snprintf(line, sizeof(line), "VG:PIN_UART_TX=%d", PIN_UART_TX);
    protocol_writeln(line);
    snprintf(line, sizeof(line), "VG:PIN_UART_RX=%d", PIN_UART_RX);
    protocol_writeln(line);
#ifdef DEBUG_HW
    if (dbg_hw_allowed()) {
      snprintf(line, sizeof(line), "VG:PIN_DBG_FIFO=%d", PIN_DBG_FIFO);
      protocol_writeln(line);
      snprintf(line, sizeof(line), "VG:PIN_DBG_MOV=%d", PIN_DBG_MOV);
      protocol_writeln(line);
      snprintf(line, sizeof(line), "VG:PIN_DBG_MOV_CONST=%d", PIN_DBG_MOV_CONST);
      protocol_writeln(line);
      snprintf(line, sizeof(line), "VG:PIN_DBG_CMD=%d", PIN_DBG_CMD);
      protocol_writeln(line);
      snprintf(line, sizeof(line), "VG:PIN_DBG_IRQ=%d", PIN_DBG_IRQ);
      protocol_writeln(line);
      snprintf(line, sizeof(line), "VG:PIN_DBG_UNDERRUN=%d", PIN_DBG_UNDERRUN);
      protocol_writeln(line);
    }
#endif
    return false;
  }

  /* --- C config --- */
  if (match_any(cmd, &rest, "CS", "ConfigSet", nullptr)) {
    static char buf[CFG_LINE_MAX];
    strncpy(buf, rest, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = 0;
    char *val = nullptr;
    const char *key = split_two(buf, &val);
    if (!key || !*key || !val || !*val) {
      protocol_error("cfg", "CS key value");
      return false;
    }
    if (!config_set_key(key, val)) {
      protocol_error("cfg", "bad key/value");
      return false;
    }
    {
      const char *krest = key;
      if (starts_cmd(key, "BUZZER_use", &krest)) {
        board_buzzer_reconfigure();
      }
      if (starts_cmd(key, "motors", &krest) || starts_cmd(key, "servos", &krest)) {
        board_gpio_init();
#ifndef HOST_TEST
        if (!pio_step_reconfigure()) {
          protocol_error("cfg", "pio");
          return false;
        }
#endif
        servo_pwm_reconfigure();
        motion_on_counts_changed();
      } else if (starts_cmd(key, "SERVO", &krest) || starts_cmd(key, "axis_min", &krest) ||
                 starts_cmd(key, "axis_max", &krest) || starts_cmd(key, "soft_min", &krest) ||
                 starts_cmd(key, "soft_max", &krest)) {
        servo_pwm_refresh();
      }
    }
    /* CS updates session for init_speed/init_accel/init_terminal/init_verbose; refresh motion. */
    motion_set_speed(sess->speed_mm_s);
    motion_set_accel(sess->accel_mm_s2);
    if (!board_config_save_to_fs()) {
      /* RAM already updated; report flash failure. */
      protocol_error("cfg", "save failed");
      return false;
    }
    return false;
  }

  if (match_any(cmd, &rest, "CR", "ConfigReset", nullptr)) {
    config_reset_to_defaults();
    board_buzzer_reconfigure();
    servo_pwm_refresh();
    motion_set_speed(sess->speed_mm_s);
    motion_set_accel(sess->accel_mm_s2);
    if (!board_config_save_to_fs()) {
      protocol_error("cfg", "save failed");
      return false;
    }
    return false;
  }

  if (match_any(cmd, &rest, "CG", "ConfigGet", nullptr)) {
    if (!*rest) {
      config_foreach(cg_dump_one, nullptr);
      return false;
    }
    char key[CFG_KEY_MAX];
    char val[CFG_VAL_MAX];
    strncpy(key, rest, sizeof(key) - 1);
    key[sizeof(key) - 1] = 0;
    for (char *p = key; *p; ++p) {
      if (*p == ' ' || *p == '\t') {
        *p = 0;
        break;
      }
    }
    if (!config_get_key(key, val, sizeof(val))) {
      protocol_error("cfg", "unknown key");
      return false;
    }
    char out[96];
    snprintf(out, sizeof(out), "CG:%s=%s\n", key, val);
    protocol_write(out);
    return false;
  }

  /* --- Special --- */
  /* ME — emergency halt. */
  if (match_any(cmd, &rest, "ME", nullptr, nullptr)) {
    if (motion_path_is_active()) {
      motion_path_abort_to_planner();
    }
    motion_halt();
    return false;
  }

  /* RB / Reboot — soft MCU reset (no power cycle). EN off first. */
  if (match_any(cmd, &rest, "RB", "Reboot", nullptr)) {
    if (motion_path_is_active()) {
      motion_path_abort_to_planner();
    }
    motion_halt();
#ifndef HOST_TEST
    rp2040.reboot();
#endif
    return false;
  }

  if (match_any(cmd, &rest, "IG", nullptr, nullptr)) {
    cmd_pinout_index();
    return false;
  }

  if (match_any(cmd, &rest, "HL", nullptr, nullptr) ||
      (cmd[0] == '?' && (cmd[1] == 0 || cmd[1] == ' ' || cmd[1] == '\t'))) {
    cmd_help();
    return false;
  }

  protocol_error("parse", "unknown command");
  return false;
}

void protocol_handle_line(const char *line) {
  static char buf[CFG_LINE_MAX];
  strncpy(buf, line, sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = 0;

  char *p = buf;
  while (*p) {
    while (*p == ' ' || *p == '\t') {
      ++p;
    }
    if (*p == 0) {
      break;
    }
    char *seg = p;
    while (*p && *p != ';') {
      ++p;
    }
    char saved = *p;
    *p = 0;
    dbg_hw_set(PIN_DBG_CMD, 1);
    bool is_wait = protocol_exec_command(seg);
    dbg_hw_set(PIN_DBG_CMD, 0);
    if (is_wait) {
      if (saved == ';') {
        protocol_set_resume_chain(p + 1);
      } else {
        protocol_set_resume_chain("");
      }
      return;
    }
    if (saved == 0) {
      break;
    }
    ++p; /* skip ';' */
  }
}
