#include "protocol_internal.h"
#include "config_store.h"
#include "motion_api.h"
#include "config_defaults.h"
#include "version.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static ProtocolIo g_io;
static char g_line[CFG_LINE_MAX];
static size_t g_len;
static char g_resume[CFG_LINE_MAX];
static bool g_have_resume;
static unsigned g_verbose_accum_ms;
static bool g_wait_active;
static ProtocolWaitKind g_wait_kind;
static bool g_wait_has_timeout;
static unsigned g_wait_timeout_ms;
static unsigned g_wait_elapsed_ms;
static float g_wait_pos_mm;
static int g_wait_pos_sign;

void protocol_set_resume_chain(const char *chain) {
  if (!chain) {
    g_resume[0] = 0;
  } else {
    strncpy(g_resume, chain, sizeof(g_resume) - 1);
    g_resume[sizeof(g_resume) - 1] = 0;
  }
  g_have_resume = true;
}

void protocol_cancel_resume_chain(void) {
  g_resume[0] = 0;
  g_have_resume = false;
}

bool protocol_is_waiting(void) { return g_wait_active; }

void protocol_cancel_waits_and_chain(void) {
  g_wait_active = false;
  g_wait_kind = PROTOCOL_WAIT_NONE;
  g_wait_has_timeout = false;
  g_wait_timeout_ms = 0;
  g_wait_elapsed_ms = 0;
  g_wait_pos_mm = 0.0f;
  g_wait_pos_sign = 1;
  protocol_cancel_resume_chain();
}

void protocol_begin_wait(ProtocolWaitKind kind, float timeout_s) {
  g_wait_kind = kind;
  g_wait_active = true;
  g_wait_elapsed_ms = 0;
  if (kind == PROTOCOL_WAIT_DELAY) {
    /* Delay always counts down; not an error timeout. */
    if (timeout_s < 0.0f) {
      timeout_s = 1.0f;
    }
    if (timeout_s > 86400.0f) {
      timeout_s = 86400.0f;
    }
    g_wait_has_timeout = true;
    g_wait_timeout_ms = (unsigned)(timeout_s * 1000.0f + 0.5f);
    if (g_wait_timeout_ms == 0) {
      g_wait_timeout_ms = 1;
    }
    return;
  }
  if (timeout_s < 0.0f) {
    g_wait_has_timeout = false;
    g_wait_timeout_ms = 0;
  } else {
    g_wait_has_timeout = true;
    if (timeout_s > 86400.0f) {
      timeout_s = 86400.0f;
    }
    g_wait_timeout_ms = (unsigned)(timeout_s * 1000.0f + 0.5f);
  }
}

void protocol_begin_wait_pos(float pos_mm, int sign, float timeout_s) {
  g_wait_pos_mm = pos_mm;
  g_wait_pos_sign = (sign >= 0) ? 1 : -1;
  protocol_begin_wait(PROTOCOL_WAIT_POS, timeout_s);
}

void protocol_write_n(const char *s, size_t n) {
  if (g_io.write && s && n) {
    g_io.write(s, n, g_io.ctx);
  }
}

void protocol_write(const char *s) {
  if (!s) {
    return;
  }
  protocol_write_n(s, strlen(s));
}

void protocol_writeln(const char *s) {
  protocol_write(s);
  protocol_write_n("\n", 1);
}

/** USB-only (never UIC UART) — used for Terminal Mode UART command sniff. */
static void protocol_write_debug_n(const char *s, size_t n) {
  if (g_io.write_debug && s && n) {
    g_io.write_debug(s, n, g_io.ctx);
  }
}

void protocol_debug(int min_level, const char *fmt, ...) {
  if (!fmt || config_get()->init_debug_level < min_level) {
    return;
  }
  char buf[128];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  if (n <= 0) {
    return;
  }
  if (n >= (int)sizeof(buf)) {
    n = (int)sizeof(buf) - 1;
  }
  protocol_write_debug_n(buf, (size_t)n);
}

typedef enum { PROTOCOL_SRC_USB = 0, PROTOCOL_SRC_UART = 1 } ProtocolSrc;

void protocol_error(const char *code, const char *text) {
  char buf[96];
  snprintf(buf, sizeof(buf), "!E:%s %s\n", code ? code : "?", text ? text : "");
  protocol_write(buf);
}

void protocol_init(ProtocolIo io) {
  g_io = io;
  g_len = 0;
  g_line[0] = 0;
  g_resume[0] = 0;
  g_have_resume = false;
  g_verbose_accum_ms = 0;
  g_wait_active = false;
  g_wait_kind = PROTOCOL_WAIT_NONE;
  g_wait_has_timeout = false;
  g_wait_timeout_ms = 0;
  g_wait_elapsed_ms = 0;
  g_wait_pos_mm = 0.0f;
  g_wait_pos_sign = 1;
  protocol_verbose_reset_dedupe();
  config_init_defaults();
  motion_init();
}

void protocol_send_banner(void) {
  /* GRBL-style ready banner: `# ` (hash + space) — distinct from `#I …` status. */
  char banner[160];
  const McConfig *c = config_get();
  const bool ax2 = config_axis2_enabled();
  const bool named = c->name[0] != 0;
  if (named && ax2) {
    snprintf(banner, sizeof(banner),
             "# %s - Slider Motion Controller V%s - 2 Axis ['$' for help]\n", c->name,
             MC_VERSION_FW);
  } else if (named) {
    snprintf(banner, sizeof(banner),
             "# %s - Slider Motion Controller V%s ['$' for help]\n", c->name, MC_VERSION_FW);
  } else if (ax2) {
    snprintf(banner, sizeof(banner),
             "# Slider Motion Controller V%s - 2 Axis ['$' for help]\n", MC_VERSION_FW);
  } else {
    snprintf(banner, sizeof(banner), "# Slider Motion Controller V%s ['$' for help]\n",
             MC_VERSION_FW);
  }
  protocol_write(banner);
}

static bool wait_condition_met(void) {
  McStatus st;
  motion_get_status(&st);
  if (g_wait_kind == PROTOCOL_WAIT_MOVING) {
    return !st.moving;
  }
  if (g_wait_kind == PROTOCOL_WAIT_HOMING) {
    return !st.homing;
  }
  if (g_wait_kind == PROTOCOL_WAIT_DELAY) {
    return false; /* completed only by elapsed time */
  }
  if (g_wait_kind == PROTOCOL_WAIT_POS) {
    if (g_wait_pos_sign >= 0) {
      return st.pos_mm >= g_wait_pos_mm;
    }
    return st.pos_mm <= g_wait_pos_mm;
  }
  if (g_wait_kind == PROTOCOL_WAIT_CRUISE) {
    if (!st.moving) {
      return true;
    }
    return protocol_state_letter() == 'M';
  }
  if (g_wait_kind == PROTOCOL_WAIT_NOT_CRUISE) {
    return protocol_state_letter() != 'M';
  }
  return true;
}

static void on_wait_complete(void) {
  char rest[CFG_LINE_MAX];
  bool have = g_have_resume;
  if (have) {
    strncpy(rest, g_resume, sizeof(rest) - 1);
    rest[sizeof(rest) - 1] = 0;
  } else {
    rest[0] = 0;
  }
  g_have_resume = false;
  g_resume[0] = 0;
  g_wait_active = false;
  g_wait_kind = PROTOCOL_WAIT_NONE;

  if (have && rest[0]) {
    protocol_handle_line(rest);
  }
}

static void on_wait_timeout(void) {
  g_have_resume = false;
  g_resume[0] = 0;
  g_wait_active = false;
  g_wait_kind = PROTOCOL_WAIT_NONE;
  protocol_error("timeout", "wait");
}

void protocol_poll(unsigned dt_ms) {
  motion_stub_tick_ms(dt_ms);

  if (g_wait_active) {
    if (g_wait_kind == PROTOCOL_WAIT_DELAY) {
      g_wait_elapsed_ms += dt_ms;
      if (g_wait_elapsed_ms >= g_wait_timeout_ms) {
        on_wait_complete();
      }
    } else if (g_wait_has_timeout) {
      g_wait_elapsed_ms += dt_ms;
      if (g_wait_elapsed_ms >= g_wait_timeout_ms) {
        on_wait_timeout();
      } else if (wait_condition_met()) {
        on_wait_complete();
      }
    } else if (wait_condition_met()) {
      on_wait_complete();
    }
  }

  if (session_get()->verbose) {
    g_verbose_accum_ms += dt_ms;
    int rate = config_get()->verbose_rate_hz;
    if (rate < 1 || rate > 200) {
      rate = CFG_DEFAULT_VERBOSE_RATE_HZ;
    }
    unsigned period = 1000u / (unsigned)rate;
    if (g_verbose_accum_ms >= period) {
      g_verbose_accum_ms = 0;
      protocol_send_verbose();
    }
  } else {
    g_verbose_accum_ms = 0;
    protocol_verbose_reset_dedupe();
  }
}

static void handle_realtime(uint8_t b) {
  if (b == '?') {
    protocol_send_status();
    return;
  }
  if (b == '!') {
    motion_stop();
    return;
  }
  if (b == 0x18) {
    motion_soft_reset();
    return;
  }
}

static void echo_byte(uint8_t b) {
  if (!session_get()->terminal) {
    return;
  }
  char c = (char)b;
  protocol_write_n(&c, 1);
}

static void feed_byte(ProtocolSrc src, uint8_t b) {
  if (b == '?' || b == '!' || b == 0x18) {
    handle_realtime(b);
    return;
  }

  if (b == '\r') {
    return;
  }

  if (b == 0x08 || b == 0x7F) {
    if (g_len > 0) {
      --g_len;
      g_line[g_len] = 0;
      if (session_get()->terminal) {
        protocol_write_n("\b \b", 3);
      }
    }
    return;
  }

  if (b == '\n') {
    g_line[g_len] = 0;
    if (session_get()->terminal) {
      protocol_write_n("\n", 1);
    }
    if (g_len > 0) {
      /* Sniff UIC→MC commands on USB before execute (Terminal Mode). */
      if (src == PROTOCOL_SRC_UART && session_get()->terminal) {
        protocol_write_debug_n(g_line, g_len);
        protocol_write_debug_n("\n", 1);
      }
      protocol_handle_line(g_line);
    }
    g_len = 0;
    g_line[0] = 0;
    return;
  }

  if (g_len + 1 >= sizeof(g_line)) {
    g_len = 0;
    g_line[0] = 0;
    protocol_error("parse", "line too long");
    return;
  }

  g_line[g_len++] = (char)b;
  g_line[g_len] = 0;
  echo_byte(b);
}

void protocol_feed_byte(uint8_t b) { feed_byte(PROTOCOL_SRC_USB, b); }

void protocol_feed_uart_byte(uint8_t b) { feed_byte(PROTOCOL_SRC_UART, b); }
