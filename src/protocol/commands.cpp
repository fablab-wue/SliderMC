#include "protocol_internal.h"
#include "config_store.h"
#include "board_config_fs.h"
#include "board.h"
#include "motion_api.h"
#include "motion_diag.h"
#include "config_defaults.h"
#include "pins.h"
#include "debug_hw.h"
#include "version.h"

#include <ctype.h>
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
    {"GS", "GetSpeed", "Session cruise mm/s"},
    {"GA", "GetAccel", "Session accel mm/s2"},
    {"GE", "GetEnable", "Driver enable state"},
    {"GT", "GetTerminal", "Terminal Mode state"},
    {"GV", "GetVerbose", "Verbose push state"},
    {"GD", "GetDebug", "USB debug level"},
    {"IM", "IsMoving", "Moving? 0|1"},
    {"IH", "IsHoming", "Homing? 0|1"},
    {"IL", "IsLimit", "At soft-limit position?"},
    {"IE", "IsError", "DRV_ERROR? 0|1"},
    {"IP", "IsPosition", "Position mm"},
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
    {"MH", "MoveHome", "Homing cycle"},
    {"MS", "MoveStop", "Soft decelerate"},
    {"X0-9", "Ext0-9", "Ext out 0|1; bare toggles"},
    {"CS", "ConfigSet", "Set persistent config key"},
    {"CG", "ConfigGet", "Get config key(s)"},
    {"W", "Wait", "Delay sec (default 1)"},
    {"WM", "WaitMoving", "Wait until move done"},
    {"WH", "WaitHoming", "Wait until home done"},
    {"VA", "VersionAbout", "About string"},
    {"VF", "VersionFW", "Firmware version"},
    {"VP", "VersionProtocol", "Protocol version"},
    {"H/HT", "Halt", "Emergency halt; EN off; cancel waits"},
    {"P", "Pins", "List pin assignments"},
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
  PinIndexRow rows[32];
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
  PIN_IX_ADD(PIN_EXT_4, "EXT_4", "Extender output 4 (X4)");
  PIN_IX_ADD(PIN_EXT_5, "EXT_5", "Extender output 5 (X5)");
  PIN_IX_ADD(PIN_EXT_6, "EXT_6", "Extender output 6 (X6)");
  PIN_IX_ADD(PIN_EXT_7, "EXT_7", "Extender output 7 (X7)");
  PIN_IX_ADD(PIN_EXT_8, "EXT_8", "Extender output 8 (X8)");
  PIN_IX_ADD(PIN_EXT_9, "EXT_9", "Extender output 9 (X9)");
  PIN_IX_ADD(PIN_DRV_STEP, "DRV_STEP", "STEP to driver");
  PIN_IX_ADD(PIN_DRV_DIR, "DRV_DIR", "DIR to driver");
  PIN_IX_ADD(PIN_DRV_EN, "DRV_EN", "Driver enable");
  PIN_IX_ADD(PIN_DRV_ERROR, "DRV_ERROR", "Driver fault / E-stop input");
  PIN_IX_ADD(PIN_SW_HOME, "SW_HOME", "Home / reference switch");
  PIN_IX_ADD(PIN_SW_LIMIT_L, "SW_LIMIT_L", "Hard limit left");
  PIN_IX_ADD(PIN_SW_LIMIT_R, "SW_LIMIT_R", "Hard limit right");
  PIN_IX_ADD(PIN_UART_TX, "UART_TX", "UART TX to UIC (1 Mbaud)");
  PIN_IX_ADD(PIN_UART_RX, "UART_RX", "UART RX from UIC (1 Mbaud)");
  PIN_IX_ADD(PIN_LED, "LED", "Status / heartbeat LED");
#ifdef DEBUG_HW
  PIN_IX_ADD(PIN_DBG_FIFO, "DBG_FIFO", "Scope: TX FIFO non-empty");
  PIN_IX_ADD(PIN_DBG_MOV, "DBG_MOV", "Scope: moving or homing");
  PIN_IX_ADD(PIN_DBG_MOV_CONST, "DBG_MOV_CONST", "Scope: cruise (equal delays)");
  PIN_IX_ADD(PIN_DBG_CMD, "DBG_CMD", "Scope: command handler busy");
  PIN_IX_ADD(PIN_DBG_IRQ, "DBG_IRQ", "Scope: PIO TX-not-full IRQ pulse");
  PIN_IX_ADD(PIN_DBG_UNDERRUN, "DBG_UNDERRUN", "Scope: FIFO underrun pulse");
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
  if (match_any(cmd, &rest, "ID", "IsDiag", nullptr) ||
      match_any(cmd, &rest, "IZ", "IsReset", nullptr)) {
    return true;
  }
  if (match_any(cmd, &rest, "VA", "VersionAbout", nullptr) ||
      match_any(cmd, &rest, "VF", "VersionFW", nullptr) ||
      match_any(cmd, &rest, "VP", "VersionProtocol", nullptr)) {
    return true;
  }
  if (match_any(cmd, &rest, "P", "Pins", nullptr) ||
      match_any(cmd, &rest, "IX", "Pinout", nullptr)) {
    return true;
  }
  if (match_any(cmd, &rest, "Help", "HL", nullptr) || starts_cmd(cmd, "$", &rest)) {
    return true;
  }
  if (match_any(cmd, &rest, "CS", "ConfigSet", nullptr) ||
      match_any(cmd, &rest, "CG", "ConfigGet", nullptr)) {
    return true;
  }
  if (match_any(cmd, &rest, "HT", "Halt", "H")) {
    return true;
  }
  /* X0..X9 / Ext0..Ext9 */
  if ((cmd[0] == 'X' || cmd[0] == 'x') && cmd[1] >= '0' && cmd[1] <= '9') {
    return true;
  }
  if ((cmd[0] == 'E' || cmd[0] == 'e') && (cmd[1] == 'X' || cmd[1] == 'x') &&
      (cmd[2] == 'T' || cmd[2] == 't') && cmd[3] >= '0' && cmd[3] <= '9') {
    return true;
  }
  return false;
}

/** Match Xn / Extn; return channel 0..9 or -1. */
static int match_ext_cmd(const char *cmd, const char **rest_out) {
  const char *rest = cmd;
  static const char *k_x[] = {"X0", "X1", "X2", "X3", "X4", "X5", "X6", "X7", "X8", "X9"};
  static const char *k_ext[] = {"Ext0", "Ext1", "Ext2", "Ext3", "Ext4",
                                "Ext5", "Ext6", "Ext7", "Ext8", "Ext9"};
  for (int i = 0; i < 10; ++i) {
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
  }

  /* --- M motion (longer prefixes before M) --- */
  if (match_any(cmd, &rest, "MT", "MoveTo", nullptr)) {
    float x;
    if (!parse_float_arg(rest, &x)) {
      protocol_error("parse", "MT args");
      return false;
    }
    if (!st.enabled) {
      protocol_error("disabled", "enable first");
      return false;
    }
    if (!motion_move_to(x)) {
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
    if (!motion_jog(-1)) {
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
    if (!motion_jog(+1)) {
      motion_get_status(&st);
      protocol_error(st.hard_limit ? "hard" : "soft", "jog rejected");
    }
    return false;
  }

  if (match_any(cmd, &rest, "MH", "MoveHome", nullptr)) {
    if (!st.enabled) {
      protocol_error("disabled", "enable first");
      return false;
    }
    if (!motion_home()) {
      protocol_error("home", "cfg");
    }
    return false;
  }

  if (match_any(cmd, &rest, "MS", "MoveStop", "Stop")) {
    motion_stop();
    return false;
  }

  if (match_any(cmd, &rest, "M", "Move", "MoveBy")) {
    float x;
    if (!parse_float_arg(rest, &x)) {
      protocol_error("parse", "M args");
      return false;
    }
    if (!st.enabled) {
      protocol_error("disabled", "enable first");
      return false;
    }
    if (!motion_move_by(x)) {
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
    reply_query_float("IP", st.pos_mm);
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

  /* --- W wait (WM/WH before bare W) --- */
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
    motion_halt();
    return false;
  }

  if (match_any(cmd, &rest, "P", "Pins", nullptr)) {
    char line[48];
    snprintf(line, sizeof(line), "P:PIN_DRV_STEP=%d", PIN_DRV_STEP);
    protocol_writeln(line);
    snprintf(line, sizeof(line), "P:PIN_DRV_DIR=%d", PIN_DRV_DIR);
    protocol_writeln(line);
    snprintf(line, sizeof(line), "P:PIN_DRV_EN=%d", PIN_DRV_EN);
    protocol_writeln(line);
    snprintf(line, sizeof(line), "P:PIN_SW_HOME=%d", PIN_SW_HOME);
    protocol_writeln(line);
    snprintf(line, sizeof(line), "P:PIN_DRV_ERROR=%d", PIN_DRV_ERROR);
    protocol_writeln(line);
    snprintf(line, sizeof(line), "P:PIN_SW_LIMIT_L=%d", PIN_SW_LIMIT_L);
    protocol_writeln(line);
    snprintf(line, sizeof(line), "P:PIN_SW_LIMIT_R=%d", PIN_SW_LIMIT_R);
    protocol_writeln(line);
    snprintf(line, sizeof(line), "P:PIN_EXT_0=%d", PIN_EXT_0);
    protocol_writeln(line);
    snprintf(line, sizeof(line), "P:PIN_EXT_1=%d", PIN_EXT_1);
    protocol_writeln(line);
    snprintf(line, sizeof(line), "P:PIN_EXT_2=%d", PIN_EXT_2);
    protocol_writeln(line);
    snprintf(line, sizeof(line), "P:PIN_EXT_3=%d", PIN_EXT_3);
    protocol_writeln(line);
    snprintf(line, sizeof(line), "P:PIN_EXT_4=%d", PIN_EXT_4);
    protocol_writeln(line);
    snprintf(line, sizeof(line), "P:PIN_EXT_5=%d", PIN_EXT_5);
    protocol_writeln(line);
    snprintf(line, sizeof(line), "P:PIN_EXT_6=%d", PIN_EXT_6);
    protocol_writeln(line);
    snprintf(line, sizeof(line), "P:PIN_EXT_7=%d", PIN_EXT_7);
    protocol_writeln(line);
    snprintf(line, sizeof(line), "P:PIN_EXT_8=%d", PIN_EXT_8);
    protocol_writeln(line);
    snprintf(line, sizeof(line), "P:PIN_EXT_9=%d", PIN_EXT_9);
    protocol_writeln(line);
    snprintf(line, sizeof(line), "P:PIN_UART_TX=%d", PIN_UART_TX);
    protocol_writeln(line);
    snprintf(line, sizeof(line), "P:PIN_UART_RX=%d", PIN_UART_RX);
    protocol_writeln(line);
#ifdef DEBUG_HW
    snprintf(line, sizeof(line), "P:PIN_DBG_FIFO=%d", PIN_DBG_FIFO);
    protocol_writeln(line);
    snprintf(line, sizeof(line), "P:PIN_DBG_MOV=%d", PIN_DBG_MOV);
    protocol_writeln(line);
    snprintf(line, sizeof(line), "P:PIN_DBG_MOV_CONST=%d", PIN_DBG_MOV_CONST);
    protocol_writeln(line);
    snprintf(line, sizeof(line), "P:PIN_DBG_CMD=%d", PIN_DBG_CMD);
    protocol_writeln(line);
    snprintf(line, sizeof(line), "P:PIN_DBG_IRQ=%d", PIN_DBG_IRQ);
    protocol_writeln(line);
    snprintf(line, sizeof(line), "P:PIN_DBG_UNDERRUN=%d", PIN_DBG_UNDERRUN);
    protocol_writeln(line);
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
