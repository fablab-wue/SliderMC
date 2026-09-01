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

static bool match_any(const char *s, const char **rest, const char *a, const char *b,
                      const char *c) {
  if (starts_cmd(s, a, rest)) {
    return true;
  }
  if (b && starts_cmd(s, b, rest)) {
    return true;
  }
  if (c && starts_cmd(s, c, rest)) {
    return true;
  }
  return false;
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

/** True for axis skip token `_` only. NOT bare '-', none, N, or *. */
static bool is_axis_skip_token(const char *s) {
  return s && s[0] == '_' && s[1] == 0;
}

/** True for SL/SR clear token `none` (case-insensitive). */
static bool is_none_token(const char *s) {
  if (!s || !*s) {
    return false;
  }
  return (s[0] == 'n' || s[0] == 'N') && (s[1] == 'o' || s[1] == 'O') &&
         (s[2] == 'n' || s[2] == 'N') && (s[3] == 'e' || s[3] == 'E') && s[4] == 0;
}

/**
 * Parse one or two float args for MT/M. Skip `_` → NAN (idle that axis).
 * Returns false on parse error. *have_second set if a 2nd token was present.
 */
static bool parse_move_args(const char *s, float *a, float *b, bool *have_second) {
  *have_second = false;
  *b = NAN;
  while (*s == ' ' || *s == '\t') {
    ++s;
  }
  if (!*s) {
    return false;
  }
  char tok1[CFG_VAL_MAX];
  char tok2[CFG_VAL_MAX];
  tok1[0] = tok2[0] = 0;
  size_t i = 0;
  while (*s && *s != ' ' && *s != '\t' && i + 1 < sizeof(tok1)) {
    tok1[i++] = *s++;
  }
  tok1[i] = 0;
  while (*s == ' ' || *s == '\t') {
    ++s;
  }
  if (*s) {
    *have_second = true;
    i = 0;
    while (*s && *s != ' ' && *s != '\t' && i + 1 < sizeof(tok2)) {
      tok2[i++] = *s++;
    }
    tok2[i] = 0;
    while (*s == ' ' || *s == '\t') {
      ++s;
    }
    if (*s) {
      return false; /* trailing junk */
    }
  }
  if (is_axis_skip_token(tok1)) {
    *a = NAN;
  } else if (!parse_float_arg(tok1, a)) {
    return false;
  }
  if (*have_second) {
    if (is_axis_skip_token(tok2)) {
      *b = NAN;
    } else if (!parse_float_arg(tok2, b)) {
      return false;
    }
  }
  return true;
}

/**
 * Parse SL/SR args: `_` = skip, `none` = store NAN, float = set value.
 * *set0/*set1 true when that axis should be written.
 */
static bool parse_window_args(const char *s, bool *set0, float *a, bool *set1, float *b,
                              bool *have_second) {
  *set0 = *set1 = false;
  *have_second = false;
  *a = *b = NAN;
  while (*s == ' ' || *s == '\t') {
    ++s;
  }
  if (!*s) {
    return false;
  }
  char tok1[CFG_VAL_MAX];
  char tok2[CFG_VAL_MAX];
  tok1[0] = tok2[0] = 0;
  size_t i = 0;
  while (*s && *s != ' ' && *s != '\t' && i + 1 < sizeof(tok1)) {
    tok1[i++] = *s++;
  }
  tok1[i] = 0;
  while (*s == ' ' || *s == '\t') {
    ++s;
  }
  if (*s) {
    *have_second = true;
    i = 0;
    while (*s && *s != ' ' && *s != '\t' && i + 1 < sizeof(tok2)) {
      tok2[i++] = *s++;
    }
    tok2[i] = 0;
    while (*s == ' ' || *s == '\t') {
      ++s;
    }
    if (*s) {
      return false;
    }
  }
  if (is_axis_skip_token(tok1)) {
    *set0 = false;
  } else if (is_none_token(tok1)) {
    *set0 = true;
    *a = NAN;
  } else if (parse_float_arg(tok1, a)) {
    *set0 = true;
  } else {
    return false;
  }
  if (*have_second) {
    if (is_axis_skip_token(tok2)) {
      *set1 = false;
    } else if (is_none_token(tok2)) {
      *set1 = true;
      *b = NAN;
    } else if (parse_float_arg(tok2, b)) {
      *set1 = true;
    } else {
      return false;
    }
  }
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

static void reply_window(const char *tag, float a, float b) {
  char fa[16], fb[16], buf[48];
  fmt_window_field(fa, sizeof(fa), a);
  if (config_axis2_enabled()) {
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
  const char *lng;
  const char *desc;
} HelpRow;

/* Column layout: %-5s %-16s %s  → total ≤ 80 when desc ≤ 57.
 * Rows are authored in group order: S → G → I → M → C → W → V → special. */
static const HelpRow k_help_rows[] = {
    {"SS", "SetSpeed", "Cruise speed mm/s"},
    {"SA", "SetAccel", "Accel mm/s2"},
    {"SE", "SetEnable", "Enable 0|1; bare toggles"},
    {"ST", "SetTerminal", "Terminal 0|1; bare toggles"},
    {"SV", "SetVerbose", "Verbose 0|1; bare toggles"},
    {"SD", "SetDebug", "USB debug level 0..5"},
    {"SL", "SetLeft", "Session soft limit left"},
    {"SR", "SetRight", "Session soft limit right"},
    {"GS", "GetSpeed", "Session cruise mm/s"},
    {"GA", "GetAccel", "Session accel mm/s2"},
    {"GE", "GetEnable", "Driver enable state"},
    {"GT", "GetTerminal", "Terminal Mode state"},
    {"GV", "GetVerbose", "Verbose push state"},
    {"GD", "GetDebug", "USB debug level"},
    {"GL", "GetLeft", "Session soft limit left"},
    {"GR", "GetRight", "Session soft limit right"},
    {"IM", "IsMoving", "Moving? 0|1"},
    {"IH", "IsHoming", "Homing? 0|1"},
    {"IL", "IsLimit", "At soft-limit position?"},
    {"IE", "IsError", "DRV_ERROR? 0|1"},
    {"IP", "IsPosition", "Position mm (1 or 2)"},
    {"IA", "IsAxis", "Axis count 1|2"},
    {"IT", "IsTarget", "Target mm or -"},
    {"IR", "IsReady", "Ready for motion? 0|1"},
    {"IW", "IsWaiting", "Wait active? 0|1"},
    {"ID", "IsDiag", "Motion diag counters"},
    {"IZ", "IsReset", "Last reset cause"},
    {"IX", "Pinout", "GPIO / name / desc table"},
    {"MT", "MoveTo", "Absolute move mm"},
    {"M", "Move", "Relative move mm"},
    {"ML", "MoveLeft", "Continuous jog -"},
    {"MR", "MoveRight", "Continuous jog +"},
    {"MJ", "MoveJoy", "Joy % of SS, signed"},
    {"MH", "MoveHome", "Homing cycle"},
    {"MS", "MoveStop", "Soft decelerate"},
    {"PC", "PathClear", "Clear path buffer"},
    {"PD", "PathData", "Append um value (-32768..32767)"},
    {"PG", "PathGo", "Play path buffer"},
    {"PN", "PathNumber", "Path sample count"},
    {"PS", "PathSlice", "Slice length us (>=1000); bare resets"},
    {"X0-3", "Ext0-3", "Ext out 0|1; bare toggles"},
    {"Z", "Buzzer", "Pulse buzzer ~0.1s"},
    {"CS", "ConfigSet", "Set persistent config key"},
    {"CR", "ConfigReset", "Reset all config to defaults"},
    {"CG", "ConfigGet", "Get config key(s)"},
    {"RB", "Reboot", "Soft MCU reset (no power cycle)"},
    {"W", "Wait", "Delay sec (default 1)"},
    {"WM", "WaitMoving", "Wait until move done"},
    {"WH", "WaitHoming", "Wait until home done"},
    {"WP", "WaitPos", "Wait until axis1 pos (or overstep)"},
    {"WC", "WaitCruise", "Wait until cruise M or idle"},
    {"WnC", "WaitNotCruise", "Wait until not cruise M"},
    {"VA", "VersionAbout", "About string"},
    {"VF", "VersionFW", "Firmware version"},
    {"VP", "VersionProtocol", "Protocol version"},
    {"VG", "VersionGPIO", "List PIN_*=GPIO lines (machine-readable, read-only)"},
    {"H/HT", "Halt", "Emergency halt; EN off; cancel waits"},
    {"$ /HL", "Help", "List all commands"},
};

static void cmd_help(void) {
  char line[88];
  protocol_writeln("Short Long             Description");
  protocol_writeln("----- ---------------- ------------------------------------------");
  for (size_t i = 0; i < sizeof(k_help_rows) / sizeof(k_help_rows[0]); ++i) {
    const HelpRow *r = &k_help_rows[i];
    snprintf(line, sizeof(line), "%-5s %-16s %s", r->sh, r->lng, r->desc);
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
  PinIndexRow rows[40];
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

  PIN_IX_ADD(PIN_EXT_0, "EXT_0", "Extender output 0 (X0)");
  PIN_IX_ADD(PIN_EXT_1, "EXT_1", "Extender output 1 (X1)");
  PIN_IX_ADD(PIN_EXT_2, "EXT_2", "Extender output 2 (X2)");
  PIN_IX_ADD(PIN_EXT_3, "EXT_3", "Extender output 3 (X3)");
  PIN_IX_ADD(PIN_DRV_STEP, "DRV_STEP", "STEP to driver");
  PIN_IX_ADD(PIN_DRV_DIR, "DRV_DIR", "DIR to driver");
  PIN_IX_ADD(PIN_DRV_EN, "DRV_EN", "Driver enable");
  PIN_IX_ADD(PIN_DRV_ERROR, "DRV_ERROR", "Driver fault / E-stop input");
  PIN_IX_ADD(PIN_SW_HOME, "SW_HOME", "Home / reference switch");
  PIN_IX_ADD(PIN_SW_LIMIT_L, "SW_LIMIT_L", "Hard limit left");
  PIN_IX_ADD(PIN_SW_LIMIT_R, "SW_LIMIT_R", "Hard limit right");
  if (config_axis2_enabled()) {
    PIN_IX_ADD(PIN_DRV_STEP2, "DRV_STEP2", "STEP axis2");
    PIN_IX_ADD(PIN_DRV_DIR2, "DRV_DIR2", "DIR axis2");
    PIN_IX_ADD(PIN_DRV_EN2, "DRV_EN2", "Enable axis2");
    PIN_IX_ADD(PIN_DRV_ERROR2, "DRV_ERROR2", "Fault / E-stop axis2");
    PIN_IX_ADD(PIN_SW_HOME2, "SW_HOME2", "Home switch axis2");
    PIN_IX_ADD(PIN_SW_LIMIT_L2, "SW_LIMIT_L2", "Hard limit left axis2");
    PIN_IX_ADD(PIN_SW_LIMIT_R2, "SW_LIMIT_R2", "Hard limit right axis2");
  }
  PIN_IX_ADD(PIN_UART_TX, "UART_TX", "UART TX to UIC (115200 baud)");
  PIN_IX_ADD(PIN_UART_RX, "UART_RX", "UART RX from UIC (115200 baud)");
  PIN_IX_ADD(PIN_LED, "LED", "Status / heartbeat LED");
  if (config_get()->buzzer_use && PIN_BUZZER != PIN_LED) {
    PIN_IX_ADD(PIN_BUZZER, "BUZZER", "Piezo pulse (Z)");
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
  if (match_any(cmd, &rest, "IE", "IsError", nullptr)) {
    return true;
  }
  if (match_any(cmd, &rest, "IA", "Axis", "IsAxis")) {
    return true;
  }
  if (match_any(cmd, &rest, "ID", "IsDiag", nullptr) ||
      match_any(cmd, &rest, "IZ", "IsReset", nullptr)) {
    return true;
  }
  if (match_any(cmd, &rest, "VA", "VersionAbout", nullptr) ||
      match_any(cmd, &rest, "VF", "VersionFW", nullptr) ||
      match_any(cmd, &rest, "VP", "VersionProtocol", nullptr)) {
    return true;
  }
  if (match_any(cmd, &rest, "VG", "VersionGPIO", nullptr) ||
      match_any(cmd, &rest, "IX", "Pinout", nullptr)) {
    return true;
  }
  if (match_any(cmd, &rest, "Help", "HL", nullptr) || starts_cmd(cmd, "$", &rest)) {
    return true;
  }
  if (match_any(cmd, &rest, "CS", "ConfigSet", nullptr) ||
      match_any(cmd, &rest, "CR", "ConfigReset", nullptr) ||
      match_any(cmd, &rest, "CG", "ConfigGet", nullptr)) {
    return true;
  }
  if (match_any(cmd, &rest, "HT", "Halt", "H")) {
    return true;
  }
  if (match_any(cmd, &rest, "RB", "Reboot", nullptr)) {
    return true;
  }
  if (match_any(cmd, &rest, "Z", "Buzzer", nullptr)) {
    return true;
  }
  /* X0..X3 / Ext0..Ext3 */
  if ((cmd[0] == 'X' || cmd[0] == 'x') && cmd[1] >= '0' && cmd[1] <= '3') {
    return true;
  }
  if ((cmd[0] == 'E' || cmd[0] == 'e') && (cmd[1] == 'X' || cmd[1] == 'x') &&
      (cmd[2] == 'T' || cmd[2] == 't') && cmd[3] >= '0' && cmd[3] <= '3') {
    return true;
  }
  return false;
}

/** Commands allowed while path-mode (PG) is active: safety/queries/stop/PD (live-move) only. */
static bool path_command_allowed(const char *cmd) {
  const char *rest = cmd;
  if (match_any(cmd, &rest, "MS", "MoveStop", "Stop")) {
    return true;
  }
  if (match_any(cmd, &rest, "HT", "Halt", "H")) {
    return true;
  }
  if (match_any(cmd, &rest, "RB", "Reboot", nullptr)) {
    return true;
  }
  if (match_any(cmd, &rest, "PD", "PathData", nullptr)) {
    return true;
  }
  if (match_any(cmd, &rest, "PN", "PathNumber", nullptr)) {
    return true;
  }
  if (match_any(cmd, &rest, "IM", "IsMoving", nullptr) ||
      match_any(cmd, &rest, "IH", "IsHoming", nullptr) ||
      match_any(cmd, &rest, "IL", "IsLimit", nullptr) ||
      match_any(cmd, &rest, "IE", "IsError", nullptr) ||
      match_any(cmd, &rest, "IP", "IsPosition", nullptr) ||
      match_any(cmd, &rest, "IA", "Axis", "IsAxis") ||
      match_any(cmd, &rest, "IT", "IsTarget", nullptr) ||
      match_any(cmd, &rest, "IR", "IsReady", nullptr) ||
      match_any(cmd, &rest, "IW", "IsWaiting", nullptr) ||
      match_any(cmd, &rest, "ID", "IsDiag", nullptr) ||
      match_any(cmd, &rest, "IZ", "IsReset", nullptr) ||
      match_any(cmd, &rest, "IX", "Pinout", nullptr)) {
    return true;
  }
  if (match_any(cmd, &rest, "GS", "GetSpeed", nullptr) ||
      match_any(cmd, &rest, "GA", "GetAccel", nullptr) ||
      match_any(cmd, &rest, "GE", "GetEnable", nullptr) ||
      match_any(cmd, &rest, "GT", "GetTerminal", nullptr) ||
      match_any(cmd, &rest, "GV", "GetVerbose", nullptr) ||
      match_any(cmd, &rest, "GD", "GetDebug", nullptr) ||
      match_any(cmd, &rest, "GL", "GetLeft", nullptr) ||
      match_any(cmd, &rest, "GR", "GetRight", nullptr)) {
    return true;
  }
  if (match_any(cmd, &rest, "VA", "VersionAbout", nullptr) ||
      match_any(cmd, &rest, "VF", "VersionFW", nullptr) ||
      match_any(cmd, &rest, "VP", "VersionProtocol", nullptr) ||
      match_any(cmd, &rest, "VG", "VersionGPIO", nullptr)) {
    return true;
  }
  if (match_any(cmd, &rest, "Help", "HL", nullptr) || starts_cmd(cmd, "$", &rest)) {
    return true;
  }
  if (match_any(cmd, &rest, "CG", "ConfigGet", nullptr)) {
    return true;
  }
  if (match_any(cmd, &rest, "Z", "Buzzer", nullptr)) {
    return true;
  }
  return false;
}

/** Match Xn / Extn; return channel 0..3 or -1. */
static int match_ext_cmd(const char *cmd, const char **rest_out) {
  const char *rest = cmd;
  static const char *k_x[] = {"X0", "X1", "X2", "X3"};
  static const char *k_ext[] = {"Ext0", "Ext1", "Ext2", "Ext3"};
  for (int i = 0; i < PIN_EXT_COUNT; ++i) {
    if (starts_cmd(cmd, k_x[i], &rest) || starts_cmd(cmd, k_ext[i], &rest)) {
      *rest_out = rest;
      return i;
    }
  }
  return -1;
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

  /* --- X / Ext extender outputs (before bare single-letter matches) --- */
  {
    int xi = match_ext_cmd(cmd, &rest);
    if (xi >= 0) {
      bool on = false;
      if (!parse_bool_arg_or_toggle(rest, board_ext_get(xi), &on)) {
        protocol_error("parse", "Xn 0|1");
        return false;
      }
      if (!board_ext_set(xi, on)) {
        protocol_error("parse", "Xn");
        return false;
      }
      return false;
    }
    /* EXT 4+ removed (PIN_EXT_COUNT=4). */
    if (starts_cmd(cmd, "X4", &rest) || starts_cmd(cmd, "X5", &rest) ||
        starts_cmd(cmd, "X6", &rest) || starts_cmd(cmd, "X7", &rest) ||
        starts_cmd(cmd, "X8", &rest) || starts_cmd(cmd, "X9", &rest) ||
        starts_cmd(cmd, "Ext4", &rest) || starts_cmd(cmd, "Ext5", &rest) ||
        starts_cmd(cmd, "Ext6", &rest) || starts_cmd(cmd, "Ext7", &rest) ||
        starts_cmd(cmd, "Ext8", &rest) || starts_cmd(cmd, "Ext9", &rest)) {
      protocol_error("parse", "Xn 0..3 only");
      return false;
    }
  }

  if (match_any(cmd, &rest, "Z", "Buzzer", nullptr)) {
    if (*rest) {
      protocol_error("parse", "Z args");
      return false;
    }
    board_buzzer_pulse();
    return false;
  }

  /* --- M motion (longer prefixes before M) --- */
  if (match_any(cmd, &rest, "MT", "MoveTo", nullptr)) {
    float a = 0.0f, b = NAN;
    bool have2 = false;
    if (!parse_move_args(rest, &a, &b, &have2)) {
      protocol_error("parse", "MT args");
      return false;
    }
    if (!st.enabled) {
      protocol_error("disabled", "enable first");
      return false;
    }
    bool ok;
    if (have2 && config_axis2_enabled()) {
      /* Dest == current → that axis idles (only the other moves). */
      McStatus cur;
      motion_get_status(&cur);
      if (!isnan(a) && fabsf(a - cur.pos_mm) < 1e-4f) {
        a = NAN;
      }
      if (!isnan(b) && fabsf(b - cur.pos_mm_2) < 1e-4f) {
        b = NAN;
      }
      ok = motion_move_to2(a, b);
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

  if (match_any(cmd, &rest, "ML", "MoveLeft", "MoveL")) {
    if (!st.enabled) {
      protocol_error("disabled", "enable first");
      return false;
    }
    int mask = 0;
    if (*rest) {
      int v = 0;
      if (!parse_int_arg(rest, &v) || v < 0 || v > 2) {
        protocol_error("parse", "ML 0|1|2");
        return false;
      }
      if (config_axis2_enabled()) {
        mask = v;
      }
    }
    if (!motion_jog(-1, mask)) {
      motion_get_status(&st);
      protocol_error(st.hard_limit ? "hard" : "soft", "jog rejected");
    }
    return false;
  }

  if (match_any(cmd, &rest, "MR", "MoveRight", "MoveR")) {
    if (!st.enabled) {
      protocol_error("disabled", "enable first");
      return false;
    }
    int mask = 0;
    if (*rest) {
      int v = 0;
      if (!parse_int_arg(rest, &v) || v < 0 || v > 2) {
        protocol_error("parse", "MR 0|1|2");
        return false;
      }
      if (config_axis2_enabled()) {
        mask = v;
      }
    }
    if (!motion_jog(+1, mask)) {
      motion_get_status(&st);
      protocol_error(st.hard_limit ? "hard" : "soft", "jog rejected");
    }
    return false;
  }

  if (match_any(cmd, &rest, "MJ", "MoveJoy", nullptr)) {
    float a = 0.0f, b = NAN;
    bool have2 = false;
    if (!parse_move_args(rest, &a, &b, &have2) || isnan(a) || (have2 && isnan(b))) {
      protocol_error("parse", "MJ args");
      return false;
    }
    if (!st.enabled) {
      protocol_error("disabled", "enable first");
      return false;
    }
    if (!config_axis2_enabled()) {
      b = NAN;
    } else if (!have2) {
      b = 0.0f;
    }
    if (!motion_joy(a, b)) {
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
      if (!parse_int_arg(rest, &axis) || (axis != 1 && axis != 2)) {
        protocol_error("parse", "MH 1|2");
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

  if (match_any(cmd, &rest, "M", "Move", "MoveBy")) {
    float a = 0.0f, b = NAN;
    bool have2 = false;
    if (!parse_move_args(rest, &a, &b, &have2)) {
      protocol_error("parse", "M args");
      return false;
    }
    if (!st.enabled) {
      protocol_error("disabled", "enable first");
      return false;
    }
    bool ok;
    if (have2 && config_axis2_enabled()) {
      McStatus cur;
      motion_get_status(&cur);
      float t0 = isnan(a) ? NAN : (cur.pos_mm + a);
      float t1 = isnan(b) ? NAN : (cur.pos_mm_2 + b);
      if (!isnan(t1) && fabsf(b) < 1e-6f) {
        t1 = NAN; /* zero delta = idle */
      }
      ok = motion_move_to2(t0, t1);
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

  if (match_any(cmd, &rest, "PD", "PathData", nullptr)) {
    float a = 0.0f, b = 0.0f;
    bool have2 = false;
    int16_t ia = 0, ib = 0;
    /* PD a [b]: skip → 0; single arg → (a, 0) */
    while (*rest == ' ' || *rest == '\t') {
      ++rest;
    }
    if (!*rest) {
      protocol_error("parse", "PD -32768..32767");
      return false;
    }
    char tok1[CFG_VAL_MAX];
    char tok2[CFG_VAL_MAX];
    tok1[0] = tok2[0] = 0;
    size_t ti = 0;
    while (*rest && *rest != ' ' && *rest != '\t' && ti + 1 < sizeof(tok1)) {
      tok1[ti++] = *rest++;
    }
    tok1[ti] = 0;
    while (*rest == ' ' || *rest == '\t') {
      ++rest;
    }
    if (*rest) {
      have2 = true;
      ti = 0;
      while (*rest && *rest != ' ' && *rest != '\t' && ti + 1 < sizeof(tok2)) {
        tok2[ti++] = *rest++;
      }
      tok2[ti] = 0;
    }
    if (is_axis_skip_token(tok1)) {
      ia = 0;
    } else {
      int v;
      if (!parse_int_arg(tok1, &v) || v < -32768 || v > 32767) {
        protocol_error("parse", "PD -32768..32767");
        return false;
      }
      ia = (int16_t)v;
    }
    if (have2) {
      if (is_axis_skip_token(tok2)) {
        ib = 0;
      } else {
        int v;
        if (!parse_int_arg(tok2, &v) || v < -32768 || v > 32767) {
          protocol_error("parse", "PD -32768..32767");
          return false;
        }
        ib = (int16_t)v;
      }
    }
    (void)a;
    (void)b;
    if (!motion_path_add2(ia, ib)) {
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
    float a = NAN, b = NAN;
    bool set0 = false, set1 = false, have2 = false;
    if (!parse_window_args(rest, &set0, &a, &set1, &b, &have2)) {
      protocol_error("parse", "SL args");
      return false;
    }
    (void)have2;
    if (!config_axis2_enabled()) {
      set1 = false;
    }
    if (!motion_set_window_left(set0, a, set1, b)) {
      protocol_error("limit", "SL");
    }
    return false;
  }

  if (match_any(cmd, &rest, "SR", "SetRight", nullptr)) {
    if (!*rest) {
      motion_reset_window_right();
      return false;
    }
    float a = NAN, b = NAN;
    bool set0 = false, set1 = false, have2 = false;
    if (!parse_window_args(rest, &set0, &a, &set1, &b, &have2)) {
      protocol_error("parse", "SR args");
      return false;
    }
    (void)have2;
    if (!config_axis2_enabled()) {
      set1 = false;
    }
    if (!motion_set_window_right(set0, a, set1, b)) {
      protocol_error("limit", "SR");
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
    reply_window("GL", session_effective_left(0), session_effective_left(1));
    return false;
  }
  if (match_any(cmd, &rest, "GR", "GetRight", nullptr)) {
    reply_window("GR", session_effective_right(0), session_effective_right(1));
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
    if (config_axis2_enabled()) {
      char buf[48];
      snprintf(buf, sizeof(buf), "IP:%.2f %.2f\n", (double)st.pos_mm, (double)st.pos_mm_2);
      protocol_write(buf);
    } else {
      reply_query_float("IP", st.pos_mm);
    }
    return false;
  }
  if (match_any(cmd, &rest, "IA", "Axis", "IsAxis")) {
    reply_query_int("IA", config_axis2_enabled() ? 2 : 1);
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
    char buf[96];
    unsigned fifo_min = d.fifo_min_level;
    if (fifo_min == 0xFFFFFFFFu) {
      fifo_min = 0;
    }
    snprintf(buf, sizeof(buf), "underrun=%lu peak_hz=%.0f overshoot=%ld fifo_min=%u",
             (unsigned long)d.underrun_count, (double)d.peak_step_hz,
             (long)d.overshoot_steps, fifo_min);
    reply_query("ID", buf);
    return false;
  }
  if (match_any(cmd, &rest, "IZ", "IsReset", nullptr)) {
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
    reply_query("IZ", reason);
    return false;
  }

  /* --- W wait (longer W* before bare W) --- */
  if (match_any(cmd, &rest, "WnC", "WaitNotCruise", nullptr)) {
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
  if (match_any(cmd, &rest, "W", "Wait", nullptr)) {
    float sec = 1.0f;
    if (*rest) {
      if (!parse_float_arg(rest, &sec) || sec < 0.0f) {
        protocol_error("parse", "W args");
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
    snprintf(line, sizeof(line), "VG:PIN_SW_HOME=%d", PIN_SW_HOME);
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
      snprintf(line, sizeof(line), "VG:PIN_SW_HOME2=%d", PIN_SW_HOME2);
      protocol_writeln(line);
      snprintf(line, sizeof(line), "VG:PIN_SW_LIMIT_L2=%d", PIN_SW_LIMIT_L2);
      protocol_writeln(line);
      snprintf(line, sizeof(line), "VG:PIN_SW_LIMIT_R2=%d", PIN_SW_LIMIT_R2);
      protocol_writeln(line);
    }
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
  /* HT / Halt / H — emergency halt (Help is Help/HL/$ only). */
  if (match_any(cmd, &rest, "HT", "Halt", "H")) {
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

  if (match_any(cmd, &rest, "IX", "Pinout", nullptr)) {
    cmd_pinout_index();
    return false;
  }

  if (match_any(cmd, &rest, "Help", "HL", nullptr) || starts_cmd(cmd, "$", &rest)) {
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

  /* Python-style comment: '#' … end of line */
  for (char *c = buf; *c; ++c) {
    if (*c == '#') {
      *c = 0;
      break;
    }
  }

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
