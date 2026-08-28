#pragma once

#include "protocol.h"

#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  PROTOCOL_WAIT_NONE = 0,
  PROTOCOL_WAIT_MOVING,
  PROTOCOL_WAIT_HOMING,
  PROTOCOL_WAIT_DELAY,
  PROTOCOL_WAIT_POS,
  PROTOCOL_WAIT_CRUISE,
  PROTOCOL_WAIT_NOT_CRUISE
} ProtocolWaitKind;

void protocol_write(const char *s);
void protocol_write_n(const char *s, size_t n);
void protocol_writeln(const char *s);
void protocol_error(const char *code, const char *text);

/** Execute one command (no ';'). Returns true if a wait was started. */
bool protocol_exec_command(const char *cmd);

/** Process a full line (may contain ';' chains). */
void protocol_handle_line(const char *line);

/**
 * Start a wait. For WM/WH/WP/WC/WnC, timeout_s < 0 means no timeout.
 * For DELAY, timeout_s is the delay duration (always counted).
 * Call protocol_set_resume_chain() for commands after the wait on the same line.
 */
void protocol_begin_wait(ProtocolWaitKind kind, float timeout_s);
/** WP: sign > 0 → done when pos_mm >= pos; sign < 0 → done when pos_mm <= pos. */
void protocol_begin_wait_pos(float pos_mm, int sign, float timeout_s);
void protocol_set_resume_chain(const char *chain);
void protocol_cancel_resume_chain(void);

/** True while any W / WM / WH / WP / WC / WnC wait is active. */
bool protocol_is_waiting(void);

/** Cancel active wait and discard remaining `;` chain (used by Halt). */
void protocol_cancel_waits_and_chain(void);

/** Clear verbose+terminal last-line dedupe cache. */
void protocol_verbose_reset_dedupe(void);

/** USB-only debug line if config init_debug_level >= min_level. Never goes to UIC UART. */
void protocol_debug(int min_level, const char *fmt, ...);

#ifdef __cplusplus
}
#endif
