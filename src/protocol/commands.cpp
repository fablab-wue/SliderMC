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
  if (c == 'x' || c == 'X') {
    return 0;
  }
  if (c == 'y' || c == 'Y') {
    return 1;
  }
  if (c == 'z' || c == 'Z') {
    return 2;
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

enum AxisArgKind { AXIS_MOVE = 0, AXIS_WINDOW, AXIS_PATH, AXIS_JOY };

typedef struct {
  float v[3];
  bool set[3];
  bool named;
} AxisParsed;

static bool axis_fitted(int ax) { return ax >= 0 && ax < config_axis_count(); }

/**
 * Dual syntax: positional (`100`, `_ _ 40`) or named (`Z100`, `X20Y50`). Not mixed.
 * MOVE: `_` → NAN. WINDOW: `_` skip, `none` → set+NAN. PATH: `_`/omit → 0 (int).
 * JOY: no `_`; named omitted axes stay unset (caller fills 0).
 */
static bool parse_axis_args(const char *s, int kind, AxisParsed *out) {
  out->v[0] = out->v[1] = out->v[2] = NAN;
  out->set[0] = out->set[1] = out->set[2] = false;
  out->named = false;
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
    return out->set[0] || out->set[1] || out->set[2];
  }

  int slot = 0;
  while (*s && slot < 3) {
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

static bool parse_move_args(const char *s, float *a, float *b, float *c, bool *have_second,
                            bool *have_third) {
  AxisParsed p;
  if (!parse_axis_args(s, AXIS_MOVE, &p)) {
    return false;
  }
  *a = p.set[0] ? p.v[0] : NAN;
  *b = p.set[1] ? p.v[1] : NAN;
  *c = p.set[2] ? p.v[2] : NAN;
  *have_second = p.set[1];
  *have_third = p.set[2];
  return true;
}

static bool parse_window_args(const char *s, bool *set0, float *a, bool *set1, float *b,
                              bool *set2, float *c, bool *have_second, bool *have_third) {
  AxisParsed p;
  if (!parse_axis_args(s, AXIS_WINDOW, &p)) {
    return false;
  }
  *set0 = p.set[0];
  *set1 = p.set[1];
  *set2 = p.set[2];
  *a = p.v[0];
  *b = p.v[1];
  *c = p.v[2];
  *have_second = p.set[1];
  *have_third = p.set[2];
  return true;
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

static void reply_window(const char *tag, float a, float b, float c) {
  char fa[16], fb[16], fc[16], buf[64];
  fmt_window_field(fa, sizeof(fa), a);
  int n = config_axis_count();
  if (n >= 3) {
    fmt_window_field(fb, sizeof(fb), b);
    fmt_window_field(fc, sizeof(fc), c);
    snprintf(buf, sizeof(buf), "%s:%s %s %s\n", tag, fa, fb, fc);
    protocol_write(buf);
  } else if (n >= 2) {
    fmt_window_field(fb, sizeof(fb), b);
    snprintf(buf, sizeof(buf), "%s:%s %s\n", tag, fa, fb);
    protocol_write(buf);
  } else {
    snprintf(buf, sizeof(buf), "%s:%s\n", tag, fa);
    protocol_write(buf);
  }
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
  char buf[CFG_LINE_MAX];
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
  float vel = st.vel_mm_s;
  int sign;
  if (st.moving && vel > 0.01f) {
    sign = 1;
  } else if (st.moving && vel < -0.01f) {
    sign = -1;
  } else {
    return false; /* dir-pause / ~0 vel: treat as idle */
  }
  if ((sign > 0 && st.pos_mm >= pos) || (sign < 0 && st.pos_mm <= pos)) {
    return false;
  }
  protocol_begin_wait_pos(pos, sign, timeout_s);
  return true;
}

typedef struct {
  const char *sh;
  const char *desc;
} HelpRow;

/* Two columns. Group order: S → G → I → M → P → E/B → C → W → V → special. */
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
    {"IP", "Is Position, mm (1, 2 or 3)"},
    {"IA", "Is Axis, count 1|2|3"},
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
    {"PC", "Path Clear, empty buffer"},
    {"PD", "Path Data, um -32768..32767"},
    {"PG", "Path Go, play buffer"},
    {"PN", "Path Number, sample count"},
    {"PS", "Path Slice, us >=1000; bare resets"},
    {"EO", "Ext Out, EO0..3 0|1; bare toggles"},
    {"BE", "Beep, pulse buzzer ~0.1s"},
    {"CS", "Config Set, persistent key"},
    {"CR", "Config Reset, all defaults"},
    {"CG", "Config Get, key(s)"},
    {"RB", "Reboot, soft MCU reset"},
    {"WT", "Wait Time, delay sec (default 1)"},
    {"WM", "Wait Moving, until move done"},
    {"WH", "Wait Homing, until home done"},
    {"WP", "Wait Pos, until axis1 pos"},
    {"WC", "Wait Cruise, until cruise or idle"},
    {"WN", "Wait Not cruise, until not M"},
    {"VA", "Version About, about string"},
    {"VF", "Version FW, firmware version"},
    {"VP", "Version Protocol, protocol version"},
    {"VG", "Version GPIO, PIN_*= lines"},
    {"HT", "Halt, EN off; cancel waits"},
    {"HL/$", "Help, list commands"},
    {"?/#", "Status now, compact line"},
    {"!/ESC", "Soft stop now"},
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

  PIN_IX_ADD(PIN_EXT_0, "EXT_0", "Extender output 0 (EO0)");
  PIN_IX_ADD(PIN_EXT_1, "EXT_1", "Extender output 1 (EO1)");
  PIN_IX_ADD(PIN_EXT_2, "EXT_2", "Extender output 2 (EO2)");
  PIN_IX_ADD(PIN_EXT_3, "EXT_3", "Extender output 3 (EO3)");
  PIN_IX_ADD(PIN_DRV_STEP, "DRV_STEP", "STEP to driver");
  PIN_IX_ADD(PIN_DRV_DIR, "DRV_DIR", "DIR to driver");
  PIN_IX_ADD(PIN_DRV_EN, "DRV_EN", "Driver enable");
  PIN_IX_ADD(PIN_DRV_ERROR, "DRV_ERROR", "Driver fault / E-stop input");
  PIN_IX_ADD(PIN_SW_LIMIT_L, "SW_LIMIT_L", "Hard limit left");
  PIN_IX_ADD(PIN_SW_LIMIT_R, "SW_LIMIT_R", "Hard limit right");
  if (config_axis2_enabled()) {
    PIN_IX_ADD(PIN_DRV_STEP2, "DRV_STEP2", "STEP axis2");
    PIN_IX_ADD(PIN_DRV_DIR2, "DRV_DIR2", "DIR axis2");
    PIN_IX_ADD(PIN_DRV_EN2, "DRV_EN2", "Enable axis2");
    PIN_IX_ADD(PIN_DRV_ERROR2, "DRV_ERROR2", "Fault / E-stop axis2");
    PIN_IX_ADD(PIN_SW_LIMIT_L2, "SW_LIMIT_L2", "Hard limit left axis2");
    PIN_IX_ADD(PIN_SW_LIMIT_R2, "SW_LIMIT_R2", "Hard limit right axis2");
  }
  if (config_axis3_enabled()) {
    PIN_IX_ADD(PIN_DRV_STEP3, "DRV_STEP3", "STEP axis3");
    PIN_IX_ADD(PIN_DRV_DIR3, "DRV_DIR3", "DIR axis3");
    PIN_IX_ADD(PIN_DRV_EN3, "DRV_EN3", "Enable axis3");
    PIN_IX_ADD(PIN_DRV_ERROR3, "DRV_ERROR3", "Fault / E-stop axis3");
    PIN_IX_ADD(PIN_SW_LIMIT_L3, "SW_LIMIT_L3", "Hard limit left axis3");
    PIN_IX_ADD(PIN_SW_LIMIT_R3, "SW_LIMIT_R3", "Hard limit right axis3");
  }
#ifdef PIN_CAMERA_CTRL
  PIN_IX_ADD(PIN_CAMERA_CTRL, "CAMERA_CTRL", "Camera control (reserved)");
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

/** Commands allowed while PIN_DRV_ERROR is asserted (diagnostics / config / halt). */
static bool emo_command_allowed(const char *cmd) {
  const char *rest = cmd;
  static const char *k[] = {"IE", "IA", "ID", "IC", "VA", "VF", "VP", "VG", "IG",
                            "HL", "CS", "CR", "CG", "HT", "RB", "BE", "EO"};
  for (size_t i = 0; i < sizeof(k) / sizeof(k[0]); ++i) {
    if (cmd_verb(cmd, k[i], &rest)) {
      return true;
    }
  }
  return cmd[0] == '$';
}

/** Commands allowed while path-mode (PG) is active: safety/queries/stop/PD (live-move) only. */
static bool path_command_allowed(const char *cmd) {
  const char *rest = cmd;
  static const char *k[] = {
      "MS", "HT", "RB", "PD", "PN", "IM", "IH", "IL", "IE", "IP", "IA", "IT",
      "IR", "IW", "ID", "IC", "IG", "GS", "GA", "GE", "GT", "GV", "GD", "GL",
      "GR", "VA", "VF", "VP", "VG", "HL", "CG", "BE"};
  for (size_t i = 0; i < sizeof(k) / sizeof(k[0]); ++i) {
    if (cmd_verb(cmd, k[i], &rest)) {
      return true;
    }
  }
  return cmd[0] == '$';
}

/** Match EO + channel; return 0..3 or -1. Rest is the optional 0|1. */
static int match_ext_cmd(const char *cmd, const char **rest_out) {
  const char *rest = cmd;
  if (!cmd_verb(cmd, "EO", &rest)) {
    return -1;
  }
  skip_ws(&rest);
  if (*rest < '0' || *rest > '3') {
    return -2; /* EO without 0..3 */
  }
  int ch = *rest - '0';
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
      protocol_error("parse", "EO 0..3");
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

  if (match_any(cmd, &rest, "BE", nullptr, nullptr)) {
    if (*rest) {
      protocol_error("parse", "BE args");
      return false;
    }
    board_buzzer_pulse();
    return false;
  }

  /* --- M motion (longer prefixes before M) --- */
  if (match_any(cmd, &rest, "MT", "MoveTo", nullptr)) {
    float a = 0.0f, b = NAN, c = NAN;
    bool have2 = false, have3 = false;
    if (!parse_move_args(rest, &a, &b, &c, &have2, &have3)) {
      protocol_error("parse", "MT args");
      return false;
    }
    if (!st.enabled) {
      protocol_error("disabled", "enable first");
      return false;
    }
    bool ok;
    if ((have2 || have3) && config_axis2_enabled()) {
      /* Dest == current → that axis idles (only the other moves). */
      McStatus cur;
      motion_get_status(&cur);
      if (!isnan(a) && fabsf(a - cur.pos_mm) < 1e-4f) {
        a = NAN;
      }
      if (!isnan(b) && fabsf(b - cur.pos_mm_2) < 1e-4f) {
        b = NAN;
      }
      if (!config_axis3_enabled()) {
        c = NAN;
      } else if (!isnan(c) && fabsf(c - cur.pos_mm_3) < 1e-4f) {
        c = NAN;
      }
      ok = motion_move_to_n(a, b, c);
    } else if (!isnan(a)) {
      ok = motion_move_to(a);
    } else {
      ok = true; /* skip-only */
    }
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
    float a = jp.set[0] ? jp.v[0] : 0.0f;
    float b = NAN, c = NAN;
    if (config_axis2_enabled()) {
      b = jp.set[1] ? jp.v[1] : 0.0f;
      if (config_axis3_enabled()) {
        c = jp.set[2] ? jp.v[2] : 0.0f;
      }
    } else if (jp.set[1] || jp.set[2]) {
      protocol_error("parse", "MJ args");
      return false;
    }
    if (!st.enabled) {
      protocol_error("disabled", "enable first");
      return false;
    }
    if (!motion_joy(a, b, c)) {
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
          axis > config_axis_count()) {
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
    float a = 0.0f, b = NAN, c = NAN;
    bool have2 = false, have3 = false;
    if (!parse_move_args(rest, &a, &b, &c, &have2, &have3)) {
      protocol_error("parse", "MB args");
      return false;
    }
    if (!st.enabled) {
      protocol_error("disabled", "enable first");
      return false;
    }
    bool ok;
    if ((have2 || have3) && config_axis2_enabled()) {
      McStatus cur;
      motion_get_status(&cur);
      float t0 = isnan(a) ? NAN : (cur.pos_mm + a);
      float t1 = isnan(b) ? NAN : (cur.pos_mm_2 + b);
      float t2 = isnan(c) ? NAN : (cur.pos_mm_3 + c);
      if (!isnan(t1) && fabsf(b) < 1e-6f) {
        t1 = NAN; /* zero delta = idle */
      }
      if (!config_axis3_enabled()) {
        t2 = NAN;
      } else if (!isnan(t2) && fabsf(c) < 1e-6f) {
        t2 = NAN;
      }
      ok = motion_move_to_n(t0, t1, t2);
    } else if (!isnan(a)) {
      ok = motion_move_by(a);
    } else {
      ok = true;
    }
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
    int16_t ia = pp.set[0] ? (int16_t)pp.v[0] : 0;
    int16_t ib = pp.set[1] ? (int16_t)pp.v[1] : 0;
    int16_t ic = pp.set[2] ? (int16_t)pp.v[2] : 0;
    if (!motion_path_add3(ia, ib, ic)) {
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
    if (!motion_path_go()) {
      protocol_error("busy", "path active");
    }
    return false;
  }

  if (match_any(cmd, &rest, "PN", "PathNumber", nullptr)) {
    reply_query_int("PN", (int)motion_path_count());
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
    float a = NAN, b = NAN, c = NAN;
    bool set0 = false, set1 = false, set2 = false, have2 = false, have3 = false;
    if (!parse_window_args(rest, &set0, &a, &set1, &b, &set2, &c, &have2, &have3)) {
      protocol_error("parse", "SL args");
      return false;
    }
    (void)have2;
    (void)have3;
    if (!config_axis2_enabled()) {
      set1 = false;
    }
    if (!config_axis3_enabled()) {
      set2 = false;
    }
    if (!motion_set_window_left(set0, a, set1, b, set2, c)) {
      protocol_error("limit", "SL");
    }
    return false;
  }

  if (match_any(cmd, &rest, "SR", "SetRight", nullptr)) {
    if (!*rest) {
      motion_reset_window_right();
      return false;
    }
    float a = NAN, b = NAN, c = NAN;
    bool set0 = false, set1 = false, set2 = false, have2 = false, have3 = false;
    if (!parse_window_args(rest, &set0, &a, &set1, &b, &set2, &c, &have2, &have3)) {
      protocol_error("parse", "SR args");
      return false;
    }
    (void)have2;
    (void)have3;
    if (!config_axis2_enabled()) {
      set1 = false;
    }
    if (!config_axis3_enabled()) {
      set2 = false;
    }
    if (!motion_set_window_right(set0, a, set1, b, set2, c)) {
      protocol_error("limit", "SR");
    }
    return false;
  }

  if (match_any(cmd, &rest, "SP", "SetPosition", nullptr)) {
    if (motion_is_busy() || motion_path_is_active()) {
      protocol_error("busy", "SP");
      return false;
    }
    float a = 0.0f, b = NAN, c = NAN;
    if (!*rest) {
      a = 0.0f;
      b = config_axis2_enabled() ? 0.0f : NAN;
      c = config_axis3_enabled() ? 0.0f : NAN;
    } else {
      bool have2 = false, have3 = false;
      if (!parse_move_args(rest, &a, &b, &c, &have2, &have3)) {
        protocol_error("parse", "SP args");
        return false;
      }
      if (!have2) {
        b = NAN;
      }
      if (!have3) {
        c = NAN;
      }
      if (!config_axis2_enabled()) {
        b = NAN;
      }
      if (!config_axis3_enabled()) {
        c = NAN;
      }
    }
    if (!motion_set_position(a, b, c)) {
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
    reply_window("GL", session_effective_left(0), session_effective_left(1),
                 session_effective_left(2));
    return false;
  }
  if (match_any(cmd, &rest, "GR", "GetRight", nullptr)) {
    reply_window("GR", session_effective_right(0), session_effective_right(1),
                 session_effective_right(2));
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
    if (n >= 3) {
      char buf[64];
      snprintf(buf, sizeof(buf), "IP:%.2f %.2f %.2f\n", (double)st.pos_mm,
               (double)st.pos_mm_2, (double)st.pos_mm_3);
      protocol_write(buf);
    } else if (n >= 2) {
      char buf[48];
      snprintf(buf, sizeof(buf), "IP:%.2f %.2f\n", (double)st.pos_mm, (double)st.pos_mm_2);
      protocol_write(buf);
    } else {
      reply_query_float("IP", st.pos_mm);
    }
    return false;
  }
  if (match_any(cmd, &rest, "IA", "Axis", "IsAxis")) {
    reply_query_int("IA", config_axis_count());
    return false;
  }
  if (match_any(cmd, &rest, "IT", "IsTarget", nullptr)) {
    motion_get_status(&st);
    if (st.has_target) {
      reply_query_float("IT", st.target_mm);
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
    snprintf(line, sizeof(line), "VG:PIN_DRV_STEP=%d", PIN_DRV_STEP);
    protocol_writeln(line);
    snprintf(line, sizeof(line), "VG:PIN_DRV_DIR=%d", PIN_DRV_DIR);
    protocol_writeln(line);
    snprintf(line, sizeof(line), "VG:PIN_DRV_EN=%d", PIN_DRV_EN);
    protocol_writeln(line);
    snprintf(line, sizeof(line), "VG:PIN_DRV_ERROR=%d", PIN_DRV_ERROR);
    protocol_writeln(line);
    snprintf(line, sizeof(line), "VG:PIN_SW_LIMIT_L=%d", PIN_SW_LIMIT_L);
    protocol_writeln(line);
    snprintf(line, sizeof(line), "VG:PIN_SW_LIMIT_R=%d", PIN_SW_LIMIT_R);
    protocol_writeln(line);
    if (config_axis2_enabled()) {
      snprintf(line, sizeof(line), "VG:PIN_DRV_STEP2=%d", PIN_DRV_STEP2);
      protocol_writeln(line);
      snprintf(line, sizeof(line), "VG:PIN_DRV_DIR2=%d", PIN_DRV_DIR2);
      protocol_writeln(line);
      snprintf(line, sizeof(line), "VG:PIN_DRV_EN2=%d", PIN_DRV_EN2);
      protocol_writeln(line);
      snprintf(line, sizeof(line), "VG:PIN_DRV_ERROR2=%d", PIN_DRV_ERROR2);
      protocol_writeln(line);
      snprintf(line, sizeof(line), "VG:PIN_SW_LIMIT_L2=%d", PIN_SW_LIMIT_L2);
      protocol_writeln(line);
      snprintf(line, sizeof(line), "VG:PIN_SW_LIMIT_R2=%d", PIN_SW_LIMIT_R2);
      protocol_writeln(line);
    }
    if (config_axis3_enabled()) {
      snprintf(line, sizeof(line), "VG:PIN_DRV_STEP3=%d", PIN_DRV_STEP3);
      protocol_writeln(line);
      snprintf(line, sizeof(line), "VG:PIN_DRV_DIR3=%d", PIN_DRV_DIR3);
      protocol_writeln(line);
      snprintf(line, sizeof(line), "VG:PIN_DRV_EN3=%d", PIN_DRV_EN3);
      protocol_writeln(line);
      snprintf(line, sizeof(line), "VG:PIN_DRV_ERROR3=%d", PIN_DRV_ERROR3);
      protocol_writeln(line);
      snprintf(line, sizeof(line), "VG:PIN_SW_LIMIT_L3=%d", PIN_SW_LIMIT_L3);
      protocol_writeln(line);
      snprintf(line, sizeof(line), "VG:PIN_SW_LIMIT_R3=%d", PIN_SW_LIMIT_R3);
      protocol_writeln(line);
    }
#ifdef PIN_CAMERA_CTRL
    snprintf(line, sizeof(line), "VG:PIN_CAMERA_CTRL=%d", PIN_CAMERA_CTRL);
    protocol_writeln(line);
#endif
    snprintf(line, sizeof(line), "VG:PIN_EXT_0=%d", PIN_EXT_0);
    protocol_writeln(line);
    snprintf(line, sizeof(line), "VG:PIN_EXT_1=%d", PIN_EXT_1);
    protocol_writeln(line);
    snprintf(line, sizeof(line), "VG:PIN_EXT_2=%d", PIN_EXT_2);
    protocol_writeln(line);
    snprintf(line, sizeof(line), "VG:PIN_EXT_3=%d", PIN_EXT_3);
    protocol_writeln(line);
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
    char buf[CFG_LINE_MAX];
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
      if (starts_cmd(key, "axis", &krest)) {
        board_gpio_init();
#ifndef HOST_TEST
        if (!pio_step_reconfigure()) {
          protocol_error("cfg", "pio");
          return false;
        }
#endif
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
  /* HT — emergency halt. */
  if (match_any(cmd, &rest, "HT", nullptr, nullptr)) {
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
      (cmd[0] == '$' && (cmd[1] == 0 || cmd[1] == ' ' || cmd[1] == '\t'))) {
    cmd_help();
    return false;
  }

  protocol_error("parse", "unknown command");
  return false;
}

void protocol_handle_line(const char *line) {
  char buf[CFG_LINE_MAX];
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
