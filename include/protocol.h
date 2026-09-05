#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * IO callbacks for protocol replies.
 * - write: UART and/or USB (command replies, verbose, status)
 * - write_debug: USB only (never the UIC UART)
 */
typedef struct {
  void (*write)(const char *data, size_t n, void *ctx);
  void (*write_debug)(const char *data, size_t n, void *ctx);
  void *ctx;
} ProtocolIo;

void protocol_init(ProtocolIo io);

/**
 * Send the GRBL-style ready banner (`# Slider Motion Controller V…`).
 * Call after unlock (`\n` on UIC UART or USB); host tests may call immediately after protocol_init.
 */
void protocol_send_banner(void);

/** Feed one RX byte from USB CDC (and host tests). Handles realtime chars immediately. */
void protocol_feed_byte(uint8_t b);

/**
 * Feed one RX byte from the UIC UART.
 * When Terminal Mode is on, complete command lines are mirrored to USB (write_debug)
 * before execution so an expert can sniff UIC→MC traffic.
 */
void protocol_feed_uart_byte(uint8_t b);

/**
 * Periodic work: verbose push (~3 Hz), complete W/WM/WH/WP/WC/WnC waits.
 * Call from a FreeRTOS task or main loop; dt_ms is elapsed ms since last poll.
 */
void protocol_poll(unsigned dt_ms);

/** Format and send a `?` status line (same compact `#…` / ` | ` format as verbose). */
void protocol_send_status(void);

/**
 * Format and send one verbose `#…` line (UIC display / heartbeat when verbose=1).
 */
void protocol_send_verbose(void);

char protocol_state_letter(void);

/** Cancel active W/WM/WH/WP/WC/WnC wait and discard remaining `;` chain (Halt / hard limit). */
void protocol_cancel_waits_and_chain(void);

#ifdef __cplusplus
}
#endif
